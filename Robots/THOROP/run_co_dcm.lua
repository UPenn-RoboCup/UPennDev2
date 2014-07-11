-----------------------------
-- Standard Initialization --
-----------------------------
local CTX, metadata = ...
dofile'include.lua'
assert(ffi, 'DCM | Please use LuaJIT :). Lua support in the near future')

----------------------
-- Required Modules --
----------------------
require'dcm'
local lD = require'libDynamixel'
local si = require'simple_ipc'
local ptable = require'util'.ptable
local input_co = require'DynamixelPacket2.ffi'.input_co
local munpack  = require'msgpack'.unpack
local vector = require'vector'

------------------
-- Config Cache --
------------------
local min_rad, max_rad = Config.servo.min_rad, Config.servo.max_rad
local is_unclamped, direction = Config.servo.is_unclamped, Config.servo.direction
local to_steps, to_radians  = Config.servo.to_steps, Config.servo.to_radians
local step_zero, step_offset = Config.servo.step_zero, Config.servo.step_offset
local m_to_j, j_to_m = Config.servo.motor_to_joint, Config.servo.joint_to_motor
local dcm_chains, name_to_bus = Config.chain, {}
local left_ft, right_ft = Config.left_ft, Config.right_ft
Config = nil

---------------
-- DCM Cache --
---------------
local cp_ptr = dcm.actuatorPtr.command_position
local tq_ptr = dcm.actuatorPtr.torque_enable
local p_ptr  = dcm.sensorPtr.position
local p_ptr_t  = dcm.tsensorPtr.position

-- Global times
local t_end, t_write, t_read, t_sleep, t_start

------------------------
-- libDynamixel Cache --
------------------------
local p_parse = lD.byte_to_number[lD.nx_registers.position[2]]
local p_parse_mx = lD.byte_to_number[lD.mx_registers.position[2]]

------------------------
-- Standard Lua Cache --
------------------------
local min, max, floor = math.min, math.max, math.floor
local schar = string.char
local sel, re, wr, get_time, usleep = unix.select, unix.read, unix.wr, unix.time, unix.usleep

-- Timeouts
local WRITE_TIMEOUT = 1 / 200
local READ_TIMEOUT = 1 / 62.5

--------------------
-- Context Import --
--------------------
local parent_ch, IS_THREAD
if CTX and not arg then
	IS_THREAD = true
	si.import_context(CTX)
end

----------------------
-- Process Metadata --
----------------------
if IS_THREAD then
	parent_ch = si.new_pair(metadata.ch_name)
end
local dcm_ch = si.new_subscriber('dcm!')

-------------------------------
-- Packet Processing Helpers --
-------------------------------
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

---------------------
-- Position Packet --
---------------------
local function parse_read_position(pkt, bus)
	-- Nothing to do if an error
	if pkt.error ~= 0 then return end
	-- Assume just reading position, for now
	local m_id = pkt.id
	local read_j_id = m_to_j[m_id]
	local read_val
	if bus.has_mx_id[m_id] then
		read_val = p_parse_mx(unpack(pkt.parameter)) 
	else
		read_val = p_parse(unpack(pkt.parameter))
	end
	local read_rad = step_to_radian(read_j_id, read_val)
	-- Set in Shared memory
	p_ptr[read_j_id - 1] = read_rad
	p_ptr_t[read_j_id - 1] = t_read
	return read_j_id, read_rad
end

-------------------------
-- General Read Packet --
-------------------------
local function parse_read_packet(pkt, bus)
	-- Nothing to do if an error
	if pkt.error ~= 0 then return end
	local reg_name = bus.read_reg
	local m_id = pkt.id
	if bus.has_mx_id[m_id] then
		reg = lD.mx_registers[reg_name]
	else
		reg = lD.nx_registers[reg_name]
	end
	if not reg then
		print("REG NOT FOUND", reg_name)
		return
	end
	local parse = lD.byte_to_number[reg[2]]
	-- Assume just reading position, for now
	local j_val, read_j_id = parse(unpack(pkt.parameter)), m_to_j[m_id]
	-- Set in Shared memory
	local ptr, ptr_t = dcm.sensorPtr[reg], dcm.tsensorPtr[reg]
	ptr[read_j_id - 1] = j_val
	ptr_t[read_j_id - 1] = t_read
	return read_j_id, j_val
end

------------------------
-- Get Force & Torque --
------------------------
local function get_ft(ft_motors, ft_config, bus)
	local status = lD.get_nx_data(ft_motors, bus)
	-- Return if receive nothing
	if not status then return end
	local s1, s2 = status[1], status[2]
	-- Need both readings for an actual FT reading
	if not s1 or not s2 then return end
	-- Make the cdata
	local ft_raw_c, ft_ptr = ffi.new'int16_t[4]'
	local ft_component_c = ffi.new'double[6]'
	local ft_readings_c = ffi.new'double[6]'
	-- Load the calibration
	local unloaded_voltage_c = ffi.new('double[6]', ft_config.unloaded)
	local calib_matrix_c = ffi.new('double[6][6]', ft_config.matrix)
	local calib_matrix_gain = ft_config.gain
	-- Lower ID has the 2 components
	if s1.id==ft_motors[1] then
		ffi.copy(ft_raw_c, s2.raw_parameter, 8)
		ft_component_c[0] = 3.3 * ft_raw_c[0] / 4095
		ft_component_c[1] = 3.3 * ft_raw_c[1] / 4095
		ft_component_c[2] = 3.3 * ft_raw_c[2] / 4095
		ft_component_c[3] = 3.3 * ft_raw_c[3] / 4095
		ffi.copy(ft_raw_c, s1.raw_parameter, 8)
		ft_component_c[4] = 3.3 * ft_raw_c[0] / 4095
		ft_component_c[5] = 3.3 * ft_raw_c[1] / 4095
	else
		ffi.copy(ft_raw_c, s2.raw_parameter, 8)
		ft_component_c[0] = 3.3 * ft_raw_c[0] / 4095
		ft_component_c[1] = 3.3 * ft_raw_c[1] / 4095
		ft_component_c[2] = 3.3 * ft_raw_c[2] / 4095
		ft_component_c[3] = 3.3 * ft_raw_c[3] / 4095
		ffi.copy(ft_raw_c, s1.raw_parameter, 8)
		ft_component_c[4] = 3.3 * ft_raw_c[0] / 4095
		ft_component_c[5] = 3.3 * ft_raw_c[1] / 4095
	end
	--[[
	print(bus.chain.name, 'VOLTAGE')
	for j=0,5 do print(ft_component_c[j]) end
	--]]
	-- New is always zeroed
	ffi.fill(ft_readings_c, ffi.sizeof(ft_readings_c))
	for i=0,5 do
		for j=0,5 do
			ft_readings_c[i] = ft_readings_c[i]
				+ calib_matrix_c[i][j]
				* (ft_component_c[j] - unloaded_voltage_c[j])
				* calib_matrix_gain
		end
	end
	return ft_readings_c
end

---------------------
-- Initial startup --
---------------------
local function entry(bus)
	-- Populate the IDs of the bus
	if bus.m_ids then
		bus:ping_verify(bus.m_ids)
	else
		bus:ping_probe()
	end
  bus.cmds_cnt = 0
	local status, s, n
	local has_mx, has_nx = false, false
	local rd_addrs = {}
	for _, m_id in ipairs(bus.m_ids) do
		local is_mx, is_nx = bus.has_mx_id[m_id], bus.has_nx_id[m_id]
		assert(is_mx or is_nx, "Unclassified motor ID "..m_id)
		-- Get the initial position
		s, status, n = nil, nil, 0
		repeat
			if is_mx then
				status = lD.get_mx_position(m_id, bus)
			else
				status = lD.get_nx_position(m_id, bus)
			end
			s = status[1]
			n = n + 1
		until s or n>5
		assert(n<5)
		assert(s.parameter, 'Bad initial')
		assert(s.id==m_id, 'bad id coherence, pos')
		t_read = get_time()
		local j_id, rad = parse_read_position(s, bus)
		cp_ptr[j_id-1] = rad
		-- Read the current torque states
		s, status, n = nil, nil, 0
		repeat
			if is_mx then
				status = lD.get_mx_torque_enable(m_id, bus)
			else
				status = lD.get_nx_torque_enable(m_id, bus)
			end
			s = status[1]
			n = n + 1
		until s or n>5
		assert(n<5)
		local tq_parse
		if is_mx then
			tq_parse = lD.byte_to_number[lD.mx_registers.torque_enable[2]]
		else
			tq_parse = lD.byte_to_number[lD.nx_registers.torque_enable[2]]
		end
		local tq_en = tq_parse(unpack(s.parameter))
		assert(s.id==m_id, 'bad id coherence, tq')
		j_id = m_to_j[s.id]
		dcm.actuatorPtr.torque_enable[j_id - 1] = tq_en
		--
		if is_nx then
			table.insert(rd_addrs, lD.nx_registers.position)
			has_nx = true
		else
			table.insert(rd_addrs, lD.mx_registers.position)
			has_mx = true
		end
	end
	-- Set the reading command for the bus and write addrs if needed
	if has_mx and has_nx then
		bus.read_cmd_str = lD.get_bulk(string.char(unpack(bus.m_ids)), rd_addrs)
	elseif has_nx then
		bus.read_cmd_str = lD.get_nx_position(bus.m_ids)
	else
		bus.read_cmd_str = lD.get_mx_position(bus.m_ids)
	end
end

-----------------------------
-- Parent command handling --
-----------------------------
local function do_parent(request, bus)
	-- Check if writing to the motors
	local wr_reg, rd_reg = request.wr_reg, request.rd_reg
	local bus_name = request.bus
	--ptable(request)
	local m_id
	if wr_reg then
		local ptr = dcm.actuatorPtr[wr_reg]
		local m_ids, m_vals, addr_n_len = {}, {}, {}
		local has_nx, has_mx = false, false
		for j_id, is_changed in pairs(request.ids) do
			m_id = j_to_m[j_id]
			if bus.has_mx_id[m_id] then
				has_mx = true
				table.insert(m_ids, m_id)
				table.insert(m_vals, ptr[j_id-1])
				table.insert(addr_n_len, lD.mx_registers[wr_reg])
				-- if TQ then copy stuff
				if wr_reg=='torque_enable' then
					cp_ptr[j_id-1] = p_ptr[j_id-1]
				end
			elseif bus.has_nx_id[m_id] then
				has_nx = true
				table.insert(m_ids, m_id)
				table.insert(m_vals, ptr[j_id-1])
				table.insert(addr_n_len, lD.nx_registers[wr_reg])
				-- if TQ then copy stuff
				if wr_reg=='torque_enable' then
					cp_ptr[j_id-1] = p_ptr[j_id-1]
				end
			end
		end
    if wr_reg=='torque_enable' then
      -- Need this to work well
      local tq_status
      for i, m_id in ipairs(m_ids) do
  			if bus.has_mx_id[m_id] then
          tq_status = lD.set_mx_torque_enable(m_id, m_vals[i], bus)
        else
          tq_status = lD.set_nx_torque_enable(m_id, m_vals[i], bus)
        end
        if #tq_status==1 then
          print("TQ", m_id)
          ptable(tq_status[1])
        else
          print("BAD TQ", m_id)
        end
      end
      -- Done the cycle
      return
    else
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
    end
	elseif rd_reg then
		-- TODO: Add bulk read if mix mx and nx
		-- FOR NOW: Just do one, and re-enqueue the other
		local j_ids = request.ids
		local m_ids, addr_n_len = {}, {}
		local has_nx, has_mx = false, false
		for _, j_id in ipairs(request.ids) do
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
    -- Check if reading position already
    if rd_reg=='position' and bus.enable_read then
			return
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
		local bus = name_to_bus[bus_name]
		if not bus then return end
		bus[request.key] = request.val
	elseif request.ft then
		if bus.name=='lleg' then
			local ft_reading = get_ft({24, 26}, left_ft, bus)
			print("FT LEFT", ft_reading)
			if not ft_reading then return end
			-- Copy into SHM
			ffi.copy(dcm.sensorPtr.lfoot, ft_reading, ffi.sizeof(ft_reading))
		elseif bus.name=='rleg' then
			local ft_reading = get_ft({23, 25}, right_ft, bus)
			print("FT RIGHT", ft_reading)
			if not ft_reading then return end
			-- Copy into SHM
			ffi.copy(dcm.sensorPtr.rfoot, ft_reading, ffi.sizeof(ft_reading))
		end
	end
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
		send_ids, commands, cmd_addrs = {}, {}, {}
		has_mx, has_nx = false, false
		for i, m_id in ipairs(m_ids) do
			is_mx = bus.has_mx_id[m_id]
			has_mx = has_mx or is_mx
			has_nx = has_nx or (not is_mx)
			j_id = m_to_j[m_id]
			-- Only ad position commands if torque enabled
			if tq_ptr[j_id-1]==1 then
				table.insert(send_ids, m_id)
				table.insert(commands, radian_to_step(j_id, cp_ptr[j_id - 1]))
				table.insert(cmd_addrs,
					is_mx and lD.mx_registers.command_position or lD.nx_registers.command_position
				)
			end
		end
		-- Perform the sync write
		if #commands>0 then
			if has_nx and has_mx then
				lD.set_bulk(schar(unpack(send_ids)), cmd_addrs, commands, bus)
			elseif has_nx then
				lD.set_nx_command_position(send_ids, commands, bus)
			else
				lD.set_nx_command_position(send_ids, commands, bus)
			end
		end
		bus.cmds_cnt = bus.cmds_cnt + 1
		coroutine.yield(0)
		-- Run the parent queue until a write to the bus
		request = table.remove(bus.request_queue)
		while request do
			n, reg = do_parent(request, bus)
			if n then
				coroutine.yield(n, reg)
				break
			end
			request = table.remove(bus.request_queue)
		end
		if bus.enable_read then
			-- Send a position read command to the bus
			bus:send_instruction(bus.read_cmd_str)
			coroutine.yield(#bus.m_ids, 'position')
		else
			-- Copy from command position
			for i, m_id in ipairs(m_ids) do
				j_id = m_to_j[m_id]
				p_ptr[j_id-1] = cp_ptr[j_id-1]
				p_ptr_t[j_id-1] = t_read
			end
		end
	end
end

local function consolidate_queue(request_queue, req)
	-- Consolidate the queue
	local is_merge = false
	for _, v in ipairs(request_queue) do
		if v.rd_reg and req.rd_reg and v.rd_reg==req.rd_reg then
			for _, id in ipairs(req.ids) do v.ids[id] = true end
			is_merge = true
			break
		elseif v.wr_reg and req.wr_reg and v.wr_reg==req.wr_reg then
			for _, id in ipairs(req.ids) do v.ids[id] = true end
			is_merge = true
			break
		end
	end
	if is_merge then return end
	-- Enqueue for the bus if not merged to previous
	table.insert(request_queue, req)
end

local function process_parent(buses)
	local requests = dcm_ch:receive(true)
	if not requests then return end
	local req
	for _, request in ipairs(requests) do
		-- Requests are messagepacked
		req = munpack(request)
		if req.ids then
			for bus_id, bus in ipairs(buses) do
				consolidate_queue(bus.request_queue, req)
			end
		end
	end
end

---------------
-- Main loop --
---------------
local function loop(buses)
	local t0 = get_time()
	local _fds = {}
	for bus_id, bus in ipairs(buses) do
		-- Add the fd
		table.insert(_fds, bus.fd)
		-- Make the output coroutine
		bus.output_co = coroutine.create(output_co)
		local status, msg = coroutine.resume(bus.output_co, bus)
    print('Starting output...', status, msg)
		bus.read_to = t0
		bus.request_queue = {}
	end
	local t_debug = t0
	local running, pkt, _co, npkt, read_reg
	local status, ready, bus, data, msg
	local sel_wait = WRITE_TIMEOUT
	local kb, debug_str
	---
	-- Begin the master loop
	---
	while true do
    t_start = get_time()
		---
		-- Check the general dcm channel
		---
		process_parent(buses)
		---
		-- Resume the write coroutines
		---
		for bus_id, bus in ipairs(buses) do
			-- Only if we are not in the read cycle
			if not bus.input_co or coroutine.status(bus.input_co)=='dead' then
				running, npkt, read_reg = coroutine.resume(bus.output_co)
				if not running then
					print('Error', bus_id, bus.name, npkt, '\n\tRestarting...')
					bus.output_co = coroutine.create(output_co)
					local running, msg = coroutine.resume(bus.output_co, bus)
          print('bus', bus_id, running, msg)
					bus.npkt_to_expect = 0
				else
					bus.npkt_to_expect = npkt
          if npkt>0 then
            bus.read_to = get_time() + READ_TIMEOUT
          end
					bus.read_reg = read_reg
				end
			end
		end
		-- Save the time of write
		t_write = get_time()
		---
		-- Resume the read coroutines, loading with data
		---
		sel_wait = WRITE_TIMEOUT
::READER::
		-- Wait for packets until we have to write again
		status, ready = sel(_fds, sel_wait)
		-- Save the time of read
		t_read = get_time()
		-- Process any reads that are left
		for bus_id, _ready in ipairs(ready) do
			bus = buses[bus_id]
			if _ready then
				-- Update the read coroutines only if ready
				if not bus.input_co or coroutine.status(bus.input_co)=='dead' then
					-- If suspended, it means within timeout
					-- TODO: This seems expensive
					bus.input_co = coroutine.create(input_co)
				end
				data = re(bus.fd)
				status, msg = coroutine.resume(bus.input_co, bus, data)
				if not status then print('DCM | In err', msg) end
			elseif bus.read_reg and t_read > bus.read_to then
				-- Not ready and out of timeout, so force a termination
				print("DCM | TIMEOUT", bus.name, bus.read_reg)
				bus.input_co = nil
				bus.read_reg = nil
			end
		end
		---
		-- Process the read coroutines
		---
		for bus_id, bus in ipairs(buses) do
			_co = bus.input_co
			if _co then
				running, pkt = coroutine.resume(_co)
				while pkt do
					if bus.read_reg=='position' then
						parse_read_position(pkt, bus)
					else
						parse_read_packet(pkt, bus)
					end
					if coroutine.status(_co)=='dead' then break end
					running, pkt = coroutine.resume(_co)
				end
			end
		end
		---
		-- Debug
		---
		if t_start - t_debug > 2 then
      os.execute('clear')
			kb = collectgarbage('count')
			debug_str = {
				string.format('\nDCM | Uptime %.2f sec, Mem: %d kB', t_start - t0, kb),
			}
			for bus_id, bus in ipairs(buses) do
				table.insert(debug_str, string.format(
					'%s Command Rate %.1f Hz', bus.name, bus.cmds_cnt / 2)
				)
				bus.cmds_cnt = 0
			end
			print(table.concat(debug_str, '\n'))
			t_debug = t_start
		end
		---
		-- Loop maintenence
		---
		collectgarbage('step')
		-- Sleep until it is the write cycle
		t_end = get_time()
		t_sleep = WRITE_TIMEOUT - (t_end - t_start)
		if t_sleep > 0 then
			sel_wait = t_sleep
			goto READER
		end
	end
end

local buses = {}
for chain_id, chain in ipairs(dcm_chains) do
	print("========\nOpening\n=======\n", chain.name, chain.ttyname)
	local bus = lD.new_bus(chain.ttyname)
	local DP2ffi = require'DynamixelPacket2.ffi' -- 2.0 protocol
	DP2ffi.init_bus(bus)
	-- Point to the configuration
	for k, v in pairs(chain) do bus[k] = v end
	-- Add to our table
	table.insert(buses, bus)
	-- Lookup table
	assert(bus.name, 'No bus name identifier!')
	name_to_bus[bus.name] = bus
	-- Initialize the bus
	entry(bus)
end

-- Begin the loop
loop(buses)
