#!/usr/bin/env luajit

-- Communication with multiple dynamixel chains --
local CTX, metadata = ...
dofile'include.lua'

-- Required Modules
require'dcm'
local lD = require'libDynamixel'
local si = require'simple_ipc'
local ptable = require'util'.ptable
local input_co = require'DynamixelPacket2.ffi'.input_co
local munpack  = require'msgpack'.unpack
local vector = require'vector'
local signal = require'signal'
local ffi = require'ffi'

-- Timeouts
local WRITE_TIMEOUT = 1 / 250
local READ_TIMEOUT = 1 / 250

-- Setup the channels
local IS_THREAD = CTX and not arg
if IS_THREAD then
	si.import_context(CTX)
	parent_ch = si.new_pair(metadata.ch_name)
end
local dcm_ch = si.new_subscriber'dcm!'

-- Config Cache
local min_rad, max_rad = Config.servo.min_rad, Config.servo.max_rad
local is_unclamped, direction = Config.servo.is_unclamped, Config.servo.direction
local to_steps, to_radians  = Config.servo.to_steps, Config.servo.to_radians
local step_zero, step_offset = Config.servo.step_zero, Config.servo.step_offset
local m_to_j, j_to_m = Config.servo.motor_to_joint, Config.servo.joint_to_motor
local dcm_chains = Config.chain

-- Gripper states
local is_gripper, gripper_mode = {}, {}
for _,id in ipairs(Config.parts.LGrip) do
	is_gripper[id] = true
	gripper_mode[id] = 0
end
for _,id in ipairs(Config.parts.RGrip) do
	is_gripper[id] = true
	gripper_mode[id] = 0
end

-- DCM Cache
local cp_ptr = dcm.actuatorPtr.command_position
local tq_en_ptr = dcm.actuatorPtr.torque_enable
local tq_ptr = dcm.actuatorPtr.command_torque
--
local p_ptr  = dcm.sensorPtr.position
local p_ptr_t = dcm.tsensorPtr.position
local c_ptr  = dcm.sensorPtr.current
local c_ptr_t = dcm.tsensorPtr.current

--------------------------
-- Global tmp variables --
--------------------------
local t_write, t_read, t_start
local t_debug, dt_debug = 0, 0
local status, ready, bus, data, msg, requests
local sel_wait, debug_str
local _fds = {}
local numbered_buses = {}
local named_buses = {}
local is_running = true

-- Clean up
local function shutdown ()
	is_running = false
end
signal.signal("SIGINT", shutdown)
signal.signal("SIGTERM", shutdown)

-- libDynamixel Cache
local p_parse = lD.byte_to_number[lD.nx_registers.position[2]]
local p_parse_mx = lD.byte_to_number[lD.mx_registers.position[2]]
local c_parse = lD.byte_to_number[lD.nx_registers.current[2]]

-- Standard Lua Cache
local min, max, floor = math.min, math.max, math.floor
local char = string.char
local sel, uread, get_time, usleep = unix.select, unix.read, unix.time, unix.usleep

-- Packet Processing Helpers
local function radian_clamp(idx, radian)
	if is_unclamped[idx] then return radian end
	return min(max(radian, min_rad[idx]), max_rad[idx])
end
local function radian_to_step(idx, radian)
	return floor(direction[idx] * radian_clamp(idx, radian) * to_steps[idx] + step_zero[idx] + step_offset[idx])
end
local function step_to_radian(idx, step)
	return direction[idx] * to_radians[idx] * (step - step_zero[idx] - step_offset[idx])
end

-- Force/Torque Data handling
local left_ft = {
	id = Config.left_ft.id,
	m_ids = Config.left_ft.m_ids,
	raw = ffi.new'uint16_t[4]',
	readings = ffi.new'double[6]',
	component = ffi.new'double[6]',
	unloaded = ffi.new('double[6]', Config.left_ft.unloaded),
	calibration_mat = ffi.new('double[6][6]', Config.left_ft.matrix),
	calibration_gain = Config.left_ft.gain,
	shm = dcm.sensorPtr.lfoot,
}
local right_ft = {
	id = Config.right_ft.id,
	m_ids = Config.right_ft.m_ids,
	raw = ffi.new'uint16_t[4]',
	readings = ffi.new'double[6]',
	component = ffi.new'double[6]',
	unloaded = ffi.new('double[6]', Config.right_ft.unloaded),
	calibration_mat = ffi.new('double[6][6]', Config.right_ft.matrix),
	calibration_gain = Config.right_ft.gain,
	shm = dcm.sensorPtr.rfoot,
}
local function parse_ft(ft, raw_str, m_id)
	-- Lower ID has the 2 components
	if m_id==ft.m_ids[1] then
		ffi.copy(ft.raw, raw_str, 8)
		ft.component[0] = 3.3 * ft.raw[0] / 4095 - ft.unloaded[0]
		ft.component[1] = 3.3 * ft.raw[1] / 4095 - ft.unloaded[1]
		ft.component[2] = 3.3 * ft.raw[2] / 4095 - ft.unloaded[2]
		ft.component[3] = 3.3 * ft.raw[3] / 4095 - ft.unloaded[3]
	elseif m_id==ft.m_ids[2] then
		ffi.copy(ft.raw, raw_str, 4)
		local raw16_as_8 = ffi.cast('uint8_t*', ft.raw)
		if m_id==23 and raw16_as_8[1]==0 then raw16_as_8[1] = 8 end
		ft.component[4] = 3.3 * ft.raw[0] / 4095 - ft.unloaded[4]
		ft.component[5] = 3.3 * ft.raw[1] / 4095 - ft.unloaded[5]
	else
		return
	end
	-- New is always zeroed
	ffi.fill(ft.readings, ffi.sizeof(ft.readings))
	for i=0,5 do
		for j=0,5 do
			ft.readings[i] = ft.readings[i]
			+ ft.calibration_mat[i][j]
			* ft.component[j]
			* ft.calibration_gain
		end
	end
	ffi.copy(ft.shm, ft.readings, ffi.sizeof(ft.readings))
end

-- Custom Leg Packet
local leg_packet_reg = {'position', 'current', 'data'}
local leg_packet_sz = 0
local leg_packet_offsets = {}
for i,v in ipairs(leg_packet_reg) do
	local reg = assert(lD.nx_registers[v])
	local sz = reg[2]
	table.insert(leg_packet_offsets, (leg_packet_offsets[1] or 0) + sz)
	leg_packet_sz = leg_packet_sz + sz
end
local function form_leg_read_cmd(bus)
	-- TODO: Verify the addresses for each leg
	assert(
	lD.check_indirect_address(bus.m_ids, leg_packet_reg, bus),
	'Bad Indirect addresses for the leg chain'
	)
	bus.read_loop_cmd_str = lD.get_indirect_data(bus.m_ids, leg_packet_reg)
	bus.read_loop_cmd_n = #bus.m_ids
	bus.read_loop_cmd = 'leg'
end
local function parse_read_leg(pkt, bus)
	-- Nothing to do if an error
	if pkt.error ~= 0 then return end
	if #pkt.parameter ~= leg_packet_sz then return end
	-- Assume just reading position, for now
	local m_id = pkt.id
	local read_j_id = m_to_j[m_id]
	-- Set Position in SHM
	local read_val = p_parse(unpack(pkt.parameter, 1, leg_packet_offsets[1]))
	local read_rad = step_to_radian(read_j_id, read_val)
	p_ptr[read_j_id - 1] = read_rad
	p_ptr_t[read_j_id - 1] = t_read
	-- Set Current in SHM
	local read_cur = c_parse(unpack(pkt.parameter, leg_packet_offsets[1]+1, leg_packet_offsets[2]))
	c_ptr[read_j_id - 1] = read_cur
	c_ptr_t[read_j_id - 1] = t_read
	-- Update the F/T Sensor
	local raw_str = pkt.raw_parameter:sub(leg_packet_offsets[2]+1, leg_packet_offsets[3])
	parse_ft(left_ft, raw_str, m_id)
	parse_ft(right_ft, raw_str, m_id)
	return read_j_id
end

-- Custom Arm Packet
local arm_packet_reg = {'position', 'current'}
local arm_packet_sz = 0
local arm_packet_offsets = {}
for i,v in ipairs(arm_packet_reg) do
	local reg = assert(lD.nx_registers[v])
	local sz = reg[2]
	table.insert(arm_packet_offsets, (arm_packet_offsets[1] or 0) + sz)
	arm_packet_sz = arm_packet_sz + sz
end
local arm_packet_reg_mx = {'position','speed','load','voltage','temperature'}
local arm_packet_sz_mx = 0
local arm_packet_offsets_mx = {}
for i,v in ipairs(arm_packet_reg_mx) do
	local reg = assert(lD.mx_registers[v])
	local sz = reg[2]
	table.insert(arm_packet_offsets_mx, (arm_packet_offsets_mx[1] or 0) + sz)
	arm_packet_sz_mx = arm_packet_sz_mx + sz
end
local function form_arm_read_cmd(bus)
	local rd_addrs, has_mx, has_nx = {}, false, false
	for _, m_id in ipairs(bus.m_ids) do
		local is_mx, is_nx = bus.has_mx_id[m_id], bus.has_nx_id[m_id]
		if is_mx then
			-- Position through temperature (NOTE: No current)
			table.insert(rd_addrs, {lD.mx_registers.position[1], arm_packet_sz_mx})
			has_mx = true
		else
			assert(
			lD.check_indirect_address({m_id}, arm_packet_reg, bus),
			'Bad Indirect addresses for the arm chain ID '..m_id
			)
			table.insert(rd_addrs, {lD.nx_registers.indirect_data[1], arm_packet_sz})
			has_nx = true
		end
	end
	-- Set the default reading command for the bus
	if has_mx and has_nx then
		bus.read_loop_cmd_str = lD.get_bulk(char(unpack(bus.m_ids)), rd_addrs)
	elseif has_nx then
		bus.read_loop_cmd_str = lD.get_indirect_data(bus.m_ids, arm_packet_reg)
	else
		-- Sync read with just MX does not work for some reason
		-- bus.read_loop_cmd_str = lD.get_mx_position(bus.m_ids)
		bus.read_loop_cmd_str = lD.get_bulk(char(unpack(bus.m_ids)), rd_addrs)
	end
	bus.read_loop_cmd_n = #bus.m_ids
	bus.read_loop_cmd = 'arm'
end
local function parse_read_arm(pkt, bus)
	-- Nothing to do if an error
	--if pkt.error ~= 0 then return end
	-- Assume just reading position, for now
	local m_id = pkt.id
	local read_j_id = m_to_j[m_id]
	if bus.has_mx_id[m_id] then
		-- Check if MX
		if #pkt.parameter==8 then
			-- Set Position in SHM
			local read_val = p_parse_mx(unpack(pkt.parameter, 1, arm_packet_offsets_mx[1]))
			local read_rad = step_to_radian(read_j_id, read_val)
			--print(m_id, 'Read val', read_val, unpack(pkt.parameter))
			p_ptr[read_j_id - 1] = read_rad
			p_ptr_t[read_j_id - 1] = t_read
			-- Set temperature (Celsius)
			dcm.sensorPtr.temperature[read_j_id - 1] = pkt.parameter[8]
			dcm.tsensorPtr.temperature[read_j_id - 1] = t_read
		end
		return read_j_id
	end
	
	if #pkt.parameter ~= arm_packet_sz then return end
	-- Set Position in SHM
	local read_val = p_parse(unpack(pkt.parameter, 1, arm_packet_offsets[1]))
	local read_rad = step_to_radian(read_j_id, read_val)
	p_ptr[read_j_id - 1] = read_rad
	p_ptr_t[read_j_id - 1] = t_read
	-- Set Current in SHM
	local read_cur = c_parse(unpack(pkt.parameter, arm_packet_offsets[1]+1, arm_packet_offsets[2]))
	c_ptr[read_j_id - 1] = read_cur
	c_ptr_t[read_j_id - 1] = t_read
	--
	return read_j_id
end

-- Position Packet
local function parse_read_position(pkt, bus)
	-- TODO: Nothing to do if an error
	--if pkt.error ~= 0 then return end
	local m_id = pkt.id
	local read_j_id = m_to_j[m_id]
	local read_val
	if bus.has_mx_id[m_id] then
		if #pkt.parameter~=lD.mx_registers.position[2] then return end
		read_val = p_parse_mx(unpack(pkt.parameter)) 
	else
		if #pkt.parameter~=lD.nx_registers.position[2] then return end
		read_val = p_parse(unpack(pkt.parameter))
	end
	local read_rad = step_to_radian(read_j_id, read_val)
	-- Set in Shared memory
	p_ptr[read_j_id - 1] = read_rad
	p_ptr_t[read_j_id - 1] = t_read
	return read_j_id, read_rad
end

-- Arbitrary Packet
local function parse_read_packet(pkt, bus)
	--if pkt.error ~= 0 then return end
	local reg_name = bus.read_reg
	if not reg_name then return end
	local m_id = pkt.id
	if bus.has_mx_id[m_id] then
		reg = lD.mx_registers[reg_name]
	else
		reg = lD.nx_registers[reg_name]
	end
	if not reg then return end
	if #pkt.parameter~=reg[2] then return end
	local parse = lD.byte_to_number[reg[2]]
	-- Assume just reading position, for now
	local j_val, read_j_id = parse(unpack(pkt.parameter)), m_to_j[m_id]
	-- Set in Shared memory
	local ptr, ptr_t = dcm.sensorPtr[reg_name], dcm.tsensorPtr[reg_name]
	if not ptr or not ptr_t then return end
	ptr[read_j_id - 1] = j_val
	ptr_t[read_j_id - 1] = t_read
	return read_j_id
end

-- Packet parsing lookup table
local parse = setmetatable({}, {
	__index = function(t, k)
		return parse_read_packet
	end
})
-- Custom packets
parse.position = parse_read_position
parse.leg = parse_read_leg
parse.arm = parse_read_arm

-- Read loop packet
local function form_read_loop_cmd(bus, cmd)
	-- leg is *not* position, but indirect now
	if bus.name:find'leg' then
		return form_leg_read_cmd(bus)
	elseif bus.name:find'arm' then
		return form_arm_read_cmd(bus)
	end
	local rd_addrs, has_mx, has_nx = {}, false, false
	for _, m_id in ipairs(bus.m_ids) do
		local is_mx, is_nx = bus.has_mx_id[m_id], bus.has_nx_id[m_id]
		if is_mx then
			table.insert(rd_addrs, lD.mx_registers.position)
			has_mx = true
		else
			table.insert(rd_addrs, lD.nx_registers.position)
			has_nx = true
		end
	end
	-- Set the default reading command for the bus
	if has_mx and has_nx then
		bus.read_loop_cmd_str = lD.get_bulk(char(unpack(bus.m_ids)), rd_addrs)
	elseif has_nx then
		bus.read_loop_cmd_str = lD.get_nx_position(bus.m_ids)
	else
		-- Sync read with just MX does not work for some reason
		-- bus.read_loop_cmd_str = lD.get_mx_position(bus.m_ids)
		bus.read_loop_cmd_str = lD.get_bulk(char(unpack(bus.m_ids)), rd_addrs)
	end
	bus.read_loop_cmd_n = #bus.m_ids
	bus.read_loop_cmd = cmd
end

-- Parent command handling
local function do_external(request, bus)
	-- Check if writing to the motors
	local wr_reg, rd_reg = request.wr_reg, request.rd_reg
	local bus_name = request.bus
	local m_id
	if wr_reg then
		local ptr = dcm.actuatorPtr[wr_reg]
		local val = request.val
		-- Ensure we have values to give
		if not ptr and not val then return end
		local m_ids, m_vals, addr_n_len = {}, {}, {}
		local has_nx, has_mx = false, false
		for j_id, is_changed in pairs(request.ids) do
			m_id = j_to_m[j_id]
			if bus.has_mx_id[m_id] then
				local value = ptr and ptr[j_id-1] or val[j_id]
				has_mx = true
				table.insert(m_ids, m_id)
				table.insert(m_vals, value)
				table.insert(addr_n_len, lD.mx_registers[wr_reg])
			elseif bus.has_nx_id[m_id] then
				has_nx = true
				table.insert(m_ids, m_id)
				table.insert(m_vals, ptr and ptr[j_id-1] or val[j_id])
				table.insert(addr_n_len, lD.nx_registers[wr_reg])
			end
		end
		if wr_reg=='torque_enable' then
			-- Need this to work well
			local status, is_mx, tq_val, j_id, pos
			for i, m_id in ipairs(m_ids) do
				is_mx = bus.has_mx_id[m_id]
				j_id = m_to_j[m_id]
				tq_val = m_vals[i]
				if is_mx then
					status = lD.set_mx_torque_enable(m_id, tq_val, bus)[1]
				else
					status = lD.set_nx_torque_enable(m_id, tq_val, bus)[1]
				end
				--if status and status.error==0 then
				if status then
					--ptable(status)
					-- Set the CP and the P
					if tq_val==1 then
						if is_mx then
							status = lD.get_mx_position(m_id, bus)[1]
						else
							status = lD.get_nx_position(m_id, bus)[1]
						end
						j_id, pos = parse_read_position(status, bus)
						if not j_id then
							print(get_time(), "BAD TORQUE ENABLE POS READ", m_id, status and status.error)
						end
						cp_ptr[j_id - 1] = p_ptr[j_id - 1]
					end
				else
					print(get_time(), "BAD TORQUE ENABLE", m_id, tq_val, status and status.error)
				end
			end
			-- Done the cycle if setting torque
			return
		elseif wr_reg=='torque_mode' then
			local status, j_id, val
			for i, m_id in ipairs(m_ids) do
				val = m_vals[i]
				j_id = m_to_j[m_id]
				status = lD['set_mx_'..wr_reg](m_id, val, bus)[1]
				if status then
					gripper_mode[j_id] = val
					-- copy the position if going to command position mode
					if m_vals[i]==0 then
						-- TODO: ASSUMES reading actively!!
						cp_ptr[j_id - 1] = p_ptr[j_id - 1]
					end
				else
					print("BAD TORQUE MODE!!")
				end
			end
		end
		-- TODO: Special code for changing torque modes

		-- Send to the bus in sync fashion, for now
		if has_nx and has_mx then
			lD.set_bulk(m_ids, addr_n_len, m_vals, bus, true)
			return 0
		elseif has_nx then
			lD['set_nx_'..wr_reg](m_ids, m_vals, bus, true)
			return 0
		elseif has_mx then
			lD['set_mx_'..wr_reg](m_ids, m_vals, bus, true)
			return 0
		end
	elseif rd_reg then
		-- Check if reading position already
		if rd_reg=='position' and bus.enable_read then return end
		-- TODO: Add bulk read if mix mx and nx
		-- FOR NOW: Just do one, and re-enqueue the other
		local j_ids = request.ids
		local m_ids, addr_n_len = {}, {}
		local has_nx, has_mx = false, false
		for j_id, is_changed in pairs(request.ids) do
			m_id = j_to_m[j_id]
			if bus.has_mx_id[m_id] then
				has_mx = true
				table.insert(m_ids, m_id)
				table.insert(addr_n_len, lD.mx_registers[rd_reg])
			elseif bus.has_nx_id[m_id] then
				has_nx = true
				table.insert(m_ids, m_id)
				table.insert(addr_n_len, lD.nx_registers[rd_reg])
			end
		end
		if has_mx and has_nx then
			lD.get_bulk(m_ids, addr_n_len, bus, true)
		elseif has_nx then
			lD['get_nx_'..rd_reg](m_ids, bus, true)
		elseif has_mx then
			lD['get_mx_'..rd_reg](m_ids, bus, true)
		end
		return #m_ids, rd_reg
	elseif bus_name then
		print("Externally set", request.key, request.val, bus_name)
		local bus = named_buses[bus_name]
		if not bus then return end
		bus[request.key] = request.val
		return
	end
end

local function form_write_command(bus, m_ids)
	local m_ids = bus.m_ids
	local send_ids, commands, cmd_addrs = {}, {}, {}
	local has_mx, has_nx = false, false
	for i, m_id in ipairs(m_ids) do
		is_mx = bus.has_mx_id[m_id]
		has_mx = has_mx or is_mx
		has_nx = has_nx or (not is_mx)
		j_id = m_to_j[m_id]
		-- Only add position commands if torque enabled
		-- TODO: Gripper should get a command_torque!
		if tq_en_ptr[j_id-1]==1 then
			table.insert(send_ids, m_id)
			if is_gripper[j_id] and gripper_mode[j_id]==1 then
				local val = min(max(tq_ptr[j_id-1], -1023), 1023)
				table.insert(commands, val < 0 and (1024 - val) or val)
				table.insert(cmd_addrs, lD.mx_registers.command_torque)
			else
				table.insert(commands, radian_to_step(j_id, cp_ptr[j_id-1]))
				table.insert(cmd_addrs,
				is_mx and lD.mx_registers.command_position or lD.nx_registers.command_position
				)
			end
		end
	end
	-- MX-only sync does not work for some reason
	if not has_mx then cmd_addrs = nil end
	return send_ids, commands, cmd_addrs
end

------------------------
-- Default Read/Write --
------------------------
local function output_co(bus)
	local m_ids = bus.m_ids
	local chain = bus.chain
	local n, reg, j_id, request
	local send_ids, commands, cmd_addrs
	local has_nx, has_mx, is_mx
	bus.cmds_cnt = 0
	coroutine.yield()
	while true do
		-- Send the position commands
		local send_ids, commands, cmd_addrs = form_write_command(bus)
		-- Perform the sync write
		if #commands>0 then
			if cmd_addrs then
				lD.set_bulk(char(unpack(send_ids)), cmd_addrs, commands, bus)
			else
				lD.set_nx_command_position(send_ids, commands, bus)
			end
			bus.cmds_cnt = bus.cmds_cnt + 1
			coroutine.yield(0)
		end
		-- Run the parent queue until a write to the bus
		request = table.remove(bus.request_queue, 1)
		while request do
			n, reg = do_external(request, bus)
			if n then
				--bus.reads_cnt = bus.reads_cnt + n
				coroutine.yield(n, reg)
				break
			end
			request = table.remove(bus.request_queue, 1)
		end
		-- Copy the command positions if reading is not enabled,
		-- otherwise, send a read instruction
		if bus.enable_read then
			bus:send_instruction(bus.read_loop_cmd_str)
			bus.read_timeout_t = get_time() + READ_TIMEOUT * bus.read_loop_cmd_n
			bus.reads_cnt = bus.reads_cnt + bus.read_loop_cmd_n
			bus.reqs_cnt = bus.reqs_cnt + 1
			coroutine.yield(bus.read_loop_cmd_n, bus.read_loop_cmd)
		else
			for i, m_id in ipairs(m_ids) do
				j_id = m_to_j[m_id]
				p_ptr[j_id-1] = cp_ptr[j_id-1]
				p_ptr_t[j_id-1] = t_read
			end
		end
	end
end

-- Consolidate the commands for a given bus
--[[
> x = {1,2,3,4,5}; i = 1
> while i<=#x do print(i, x[i], #x); if x[i]==3 then print("rem",table.remove(x,i)) else i=i+1 end end
--]]
local function consolidate(queue)
	local i, req0, req1, ids0 = 2
	while i <= #queue do
		req0 = queue[i-1]
		req1 = queue[i]
		if (req0.rd_reg and req0.rd_reg==req1.rd_reg) or 
			(req0.wr_reg and req0.wr_reg==req1.wr_reg) 
			then
				table.remove(queue, i)
				ids0 = req0.ids
				for _, id in ipairs(req1.ids) do ids0[id] = true end
			else
				i = i + 1
			end
		end
	end

	-- Listen for command packets in non-blocking mode
	local function process_external()
		local requests = dcm_ch:receive(true)
		if not requests then return end
		local req, queue
		for _, request in ipairs(requests) do
			req = munpack(request)
			for bname, bus in pairs(named_buses) do
				queue = bus.request_queue
				table.insert(queue, req)
				--if req.ids then consolidate(queue) end
			end
		end
	end

	-- Initialize a bus object with useful variables
	local function initialize(bus)
		bus.n_read_timeouts = 0
		bus.read_timeout_t = 0
		bus.npkt_to_expect = 0
		bus.request_queue = {}
		bus.cmds_cnt = 0
		bus.reads_cnt = 0
		bus.reqs_cnt = 0
		-- Add the fd
		table.insert(_fds, bus.fd)
		-- Populate the IDs of the bus
		if bus.m_ids then
			bus:ping_verify(bus.m_ids)
		else
			bus:ping_probe()
		end
		local status, n
		local has_mx, has_nx = false, false
		for _, m_id in ipairs(bus.m_ids) do
			local is_mx, is_nx = bus.has_mx_id[m_id], bus.has_nx_id[m_id]
			assert(is_mx or is_nx, "Unclassified motor ID "..m_id)
			-- Get the initial position
			n = 0
			repeat
				if is_mx then
					status = lD.get_mx_position(m_id, bus)[1]
				else
					status = lD.get_nx_position(m_id, bus)[1]
				end
				--if status and status.error==0 then break end
				if status then break end
				n = n + 1
			until n > 5
			assert(n<=5, 'Too many attempts at reading position')
			assert(status.id==m_id, 'Bad id coherence, position')
			t_read = get_time()
			local j_id, rad = parse_read_position(status, bus)
			assert(j_id, 'Bad pos read in initialize')
			cp_ptr[j_id-1] = rad
			-- Read the current torque states
			n = 0
			repeat
				if is_mx then
					status = lD.get_mx_torque_enable(m_id, bus)[1]
				else
					status = lD.get_nx_torque_enable(m_id, bus)[1]
				end
				--if status and status.error==0 then break end
				if status then break end
				n = n + 1
			until n > 5
			assert(n<=5, 'Too many attempts at reading torque enable')
			assert(status.id==m_id, 'Bad id coherence, torque enable')
			j_id = m_to_j[m_id]
			local tq_parse
			if is_mx then
				tq_parse = lD.byte_to_number[lD.mx_registers.torque_enable[2]]
			else
				tq_parse = lD.byte_to_number[lD.nx_registers.torque_enable[2]]
			end
			tq_en_ptr[j_id-1] = tq_parse(unpack(status.parameter))
			-- Get the torque mode if the gripper
			if is_gripper[j_id] then
				n = 0
				repeat
					status = lD.get_mx_torque_mode(m_id, bus)[1]
					if status then break end
					n = n + 1
				until n > 5
				assert(n <= 5, 'Too many attempts at reading torque mode')
				local parse = lD.byte_to_number[lD.mx_registers.torque_mode[2]]
				gripper_mode[j_id] = parse(unpack(status.parameter))
			end
		end
		-- Set the default reading command for the bus
		form_read_loop_cmd(bus, 'position')
	end

	for chain_id, chain in ipairs(dcm_chains) do
		print("========\nOpening", chain.name, chain.ttyname)
		local bus = lD.new_bus(chain.ttyname)
		-- Point to the configuration
		for k, v in pairs(chain) do bus[k] = v end
		-- Lookup tables
		assert(bus.name, 'No bus name identifier!')
		named_buses[bus.name] = bus
		--table.insert(numbered_buses, bus)
		numbered_buses[bus.fd] = bus
		initialize(bus)
		-- Make the output coroutine
		bus.output_co = coroutine.wrap(output_co)
		bus.output_co(bus)
		-- Make the input coroutine
		bus.input_co = coroutine.wrap(input_co)
		bus.input_co(bus)
	end

	local t0 = get_time()
	-- Begin the master loop
	while is_running do
		t_start = get_time()
		-- Check for commands for the DCM from external sources
		process_external()
		-- Resume the write coroutines
		t_write = get_time()
		for bname, bus in pairs(named_buses) do
			-- Check if we await packets and have expired the timeout
			if bus.npkt_to_expect > 0 and t_write > bus.read_timeout_t then
				bus.n_read_timeouts = bus.n_read_timeouts + bus.npkt_to_expect
				bus.read_reg = nil
				bus.npkt_to_expect = 0
				bus.input_co(false)
			end
			-- Check if we are not expecting any packets
			-- We now output to the bus
			-- We may output a read request
			if bus.npkt_to_expect < 1 then
				bus.npkt_to_expect, bus.read_reg = bus.output_co()
			end
		end
		-- Load data from the bus into the input coroutine
		-- Loop until the write timeout is expired
		do_collect = true
		sel_wait = WRITE_TIMEOUT
		while sel_wait > 0 do
			status, ready = sel(_fds, sel_wait)
			if status==0 then break end
			-- Read in packets if we received data from the bus
			t_read = get_time()
			local pkts, rxi
			for bnum, is_ready in pairs(ready) do
				if is_ready then
					bus = numbered_buses[bnum]
					-- Place the data into packet structs
					pkts, rxi = bus.input_co(uread(bus.fd))
					-- Parse the packets into shared memory
					bus.npkt_to_expect = bus.npkt_to_expect - #pkts
					for _, pkt in ipairs(pkts) do
						parse[bus.read_reg](pkt, bus, bus)
					end
				end
			end
			-- Loop maintenence
			if do_collect then
				do_collect = false
				collectgarbage'step'
			end
			sel_wait = WRITE_TIMEOUT - (get_time() - t_start)
		end
		-- Debug messages for the user
		dt_debug = t_start - t_debug
		if dt_debug > 2 then
			t_debug = t_start
			debug_str = {
				string.format('\nDCM | Uptime %.2f sec, Mem: %d kB', t_start - t0, collectgarbage('count')),
			}
			for bname, bus in pairs(named_buses) do
				table.insert(debug_str,
				string.format('%s Command @ %.1f Hz | Read @ %.1f Hz [%d / %d timeouts]',
				bname, bus.cmds_cnt / dt_debug, bus.reqs_cnt / dt_debug, bus.n_read_timeouts, bus.reads_cnt))
				bus.reads_cnt = 0
				bus.cmds_cnt = 0
				bus.reqs_cnt = 0
				bus.n_read_timeouts = 0
			end
			debug_str = table.concat(debug_str, '\n')
			print(debug_str)
		end
	end

	-- Exit
	for bname, bus in pairs(named_buses) do
		bus:close()
	end
