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
local vector = require'vector'
local input_co = require'DynamixelPacket2.ffi'.input_co
local munpack  = require'msgpack'.unpack

------------------
-- Config Cache --
------------------
local min_rad, max_rad, is_unclamped = Config.servo.min_rad, Config.servo.max_rad, Config.servo.is_unclamped
local direction, to_steps, step_zero = Config.servo.direction, Config.servo.to_steps, Config.servo.step_offset
local to_radians, step_offset, m_to_j = Config.servo.to_radians, Config.servo.step_zero, Config.servo.motor_to_joint
local j_to_m = Config.servo.joint_to_motor
local dcm_chains, name_to_chain = Config.chain, {}
local left_ft, right_ft = Config.left_ft, Config.right_ft
Config = nil

---------------
-- DCM Cache --
---------------
local cp_ptr = dcm.actuatorPtr.command_position
local tq_ptr = dcm.actuatorPtr.torque_enable
local p_ptr  = dcm.sensorPtr.position

------------------------
-- libDynamixel Cache --
------------------------
local registers_sensor = lD.registers_sensor
-- NX
local cp_cmd = lD.set_nx_command_position
local p_read = lD.get_nx_position
local p_parse = lD.byte_to_number[lD.nx_registers.position[2]]
local tq_read = lD.get_nx_torque_enable
local tq_parse = lD.byte_to_number[lD.nx_registers.torque_enable[2]]
-- MX
local cp_cmd_mx = lD.set_mx_command_position
local p_read_mx = lD.get_mx_position
local p_parse_mx = lD.byte_to_number[lD.mx_registers.position[2]]
local tq_read_mx = lD.get_mx_torque_enable
local tq_parse_mx = lD.byte_to_number[lD.mx_registers.torque_enable[2]]

------------------------
-- Standard Lua Cache --
------------------------
local max = math.max
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
-- Clamp the radian within the min and max
-- Used when sending packets and working with actuator commands
local function radian_clamp(idx, radian)
	if is_unclamped[idx] then return radian end
	return math.min(math.max(radian, min_rad[idx]), max_rad[idx])
end
-- Radian to step, using offsets and biases
local function radian_to_step(idx, radian)
	return math.floor(direction[idx] * radian_clamp(idx, radian) * to_steps[idx] + step_zero[idx] + step_offset[idx])
end
-- Step to radian
local function step_to_radian(idx, step)
	return direction[idx] * to_radians[idx] * (step - step_zero[idx] - step_offset[idx])
end

---------------------
-- Position Packet --
---------------------
local function parse_read_position(pkt, IS_MX)
	-- Nothing to do if an error
	if pkt.error ~= 0 then return end
	-- Assume just reading position, for now
	local read_j_id = m_to_j[pkt.id]
	local read_val
	if IS_MX then
		read_val = p_parse(unpack(pkt.parameter)) 
	else
		read_val = p_parse(unpack(pkt.parameter))
	end
	local read_rad = step_to_radian(read_j_id, read_val)
	-- Set in Shared memory
	p_ptr[read_j_id - 1] = read_rad
	return read_j_id, read_rad
end

-------------------------
-- General Read Packet --
-------------------------
local function parse_read_packet(pkt, reg_name, IS_MX)
	-- Nothing to do if an error
	if pkt.error ~= 0 then return end
	local reg
	if IS_MX then
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
	local j_val, read_j_id = parse(unpack(pkt.parameter)), m_to_j[pkt.id]
	-- Set in Shared memory
	local ptr = dcm.sensorPtr[reg]
	ptr[read_j_id - 1] = j_val
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
	-- Populate the m_ids in the bus
	bus:ping_probe()
	bus.has_m = {}
	local IS_MX = bus.chain.is_mx
	for _, m_id in ipairs(bus.m_ids) do
		bus.has_m[m_id] = true
		local status, s = {}
		while not s do
			if IS_MX then
				status = p_read_mx(m_id, bus)
			else
				status = p_read(m_id, bus)
			end
			s = status[1]
		end
		assert(s.parameter, 'Bad initial')
		assert(s.id==m_id, 'bad id coherence, pos')
		local j_id, rad = parse_read_position(s, IS_MX)
		cp_ptr[j_id-1] = rad
		-- Read the current torque states
		s = nil
		while not s do
			if IS_MX then
				status = tq_read_mx(m_id, bus)
			else
				status = tq_read(m_id, bus)
			end
			s = status[1]
		end
		local tq_en
		if IS_MX then
			tq_en = tq_parse_mx(unpack(s.parameter))
		else
			tq_en = tq_parse(unpack(s.parameter))
		end
		assert(s.id==m_id, 'bad id coherence, tq')
		j_id = m_to_j[s.id]
		tq_ptr[j_id - 1] = tq_en
	end
end

-----------------------------
-- Parent command handling --
-----------------------------
local function do_parent(request, bus)
	-- Check if writing to the motors
	local wr_reg, rd_reg = request.wr_reg, request.rd_reg
	local chain_name = request.chain_name
	local m_id
	if wr_reg then
		local ptr = dcm.actuatorPtr[wr_reg]
		local m_ids, vals = {}, {}
		for _, j_id in ipairs(request.ids) do
			m_id = j_to_m[j_id]
			if bus.has_m[m_id] then
				table.insert(m_ids, m_id)
				table.insert(vals, ptr[j_id-1])
				-- if TQ then copy stuff
				if wr_reg=='torque_enable' then
					cp_ptr[j_id-1] = p_ptr[j_id-1]
				end
			end
		end
		-- Send to the bus in sync fashion, for now
		if bus.chain.is_mx then
			lD['set_mx_'..wr_reg](m_ids, vals, bus, true)
		else
			lD['set_nx_'..wr_reg](m_ids, vals, bus, true)
		end
		coroutine.yield(0)
	elseif rd_reg then
		local j_ids = request.ids
		local m_ids = {}
		for _, j_id in ipairs(request.ids) do
			m_id = j_to_m[j_id]
			if bus.has_m[m_id] then
				table.insert(m_ids, m_id)
			end
		end
		if bus.chain.is_mx then
			lD['get_mx_'..rd_reg](m_ids, bus, true)
		else
			lD['get_nx_'..rd_reg](m_ids, bus, true)
		end
		coroutine.yield(#m_ids, rd_reg)
	elseif chain_name then
		local chain = name_to_chain[chain_name]
		if chain then
			chain[request.key] = request.val
		end
	elseif request.ft then
		if bus.chain.name=='lleg' then
			local ft_reading = get_ft({24, 26}, left_ft, bus)
			print("FT LEFT", ft_reading)
			-- Copy into SHM
			if ft_reading then
				ffi.copy(dcm.sensorPtr.lfoot, ft_reading, ffi.sizeof(ft_reading))
			end
			coroutine.yield(0)
		elseif bus.chain.name=='rleg' then
			local ft_reading = get_ft({23, 25}, right_ft, bus)
			print("FT RIGHT", ft_reading)
			-- Copy into SHM
			if ft_reading then
				ffi.copy(dcm.sensorPtr.rfoot, ft_reading, ffi.sizeof(ft_reading))
			end
			coroutine.yield(0)
		end
	end
end

------------------------
-- Default Read/Write --
------------------------
local function output_co(bus)
	local m_ids = bus.m_ids
	local commands = {}
	local chain = bus.chain
	local IS_MX = chain.is_mx
	coroutine.yield()
	bus.cmds_t0 = get_time()
	bus.cmds_cnt = 0
	while true do
		-- Send the position commands
		for i, m_id in ipairs(m_ids) do
			local j_id = m_to_j[m_id]
			commands[i] = radian_to_step(j_id, cp_ptr[j_id - 1])
		end
		-- Perform the sync write
		if IS_MX then
			cp_cmd_mx(m_ids, commands, bus)
		else
			cp_cmd(m_ids, commands, bus)
		end
		bus.cmds_cnt = bus.cmds_cnt + 1
		coroutine.yield(0)
		-- Either read the position or perform a parent command
		while #bus.parent_queue>0 do
			local parent_cmd = table.remove(bus.parent_queue)
			local n, reg = do_parent(parent_cmd, bus)
			if n then
				coroutine.yield(n, reg)
				break
			end
		end
		if chain.enable_read then
			-- Send a position read command to the bus
			if IS_MX then
				p_read_mx(m_ids, bus, true)
			else
				p_read(m_ids, bus, true)
			end
			-- Set the timeout
			bus.read_to = get_time() + READ_TIMEOUT
			coroutine.yield(#m_ids, 'position')
		else
			-- Copy from command position
			for i, m_id in ipairs(m_ids) do
				j_id = m_to_j[m_id]
				p_ptr[j_id-1] = cp_ptr[j_id-1]
			end
		end
	end
end

---------------
-- Main loop --
---------------
local function loop(buses)
	local _fds = {}
	for bus_id, bus in ipairs(buses) do
		-- Add the fd
		table.insert(_fds, bus.fd)
		-- Make the output coroutine
		bus.output_co = coroutine.create(output_co)
		coroutine.resume(bus.output_co, bus)
		bus.read_to = 0
		bus.parent_queue = {}
	end
	local t0 = get_time()
	local t_debug = t0
	local t_end, t_write, t_read
	local running, pkt, _co
	local npkt, read_reg, requests, req
	-- Begin the master loop
	while true do
		-- Check the general dcm channel
		requests = dcm_ch:receive(true)
		if requests then
			for _, request in ipairs(requests) do
				req = munpack(request)
				for bus_id, bus in ipairs(buses) do
					-- Queue for the bus
					table.insert(bus.parent_queue, req)
				end
			end
		end
		-- Resume the write coroutines
		for bus_id, bus in ipairs(buses) do
			-- Only if we are not in the read cycle
			if not bus.input_co or coroutine.status(bus.input_co)=='dead' then
				running, npkt, read_reg = coroutine.resume(bus.output_co)
				if not running then
					print(bus_id, bus.name, 'Error', npkt, '\n\tRestarting...')
					bus.output_co = coroutine.create(output_co)
					coroutine.resume(bus.output_co, bus)
					bus.npkt_to_expect = 0
				else
					bus.npkt_to_expect = npkt
					bus.read_reg = read_reg
				end
			end
		end
		-- Save the timeof write
		t_write = get_time()
		-- Resume the read coroutines
		-- Wait for packets until we have to write again
		local status, ready = sel(_fds, WRITE_TIMEOUT)
		-- Save the time of read
		t_read = get_time()
		-- Process any reads that are left
		for bus_id, _ready in ipairs(ready) do
			local bus = buses[bus_id]
			if _ready then
				-- Update the read coroutines only if ready
				if not bus.input_co or coroutine.status(bus.input_co)=='dead' then
					-- If suspended, it means within timeout
					bus.input_co = coroutine.create(input_co)
				end
				local data = re(bus.fd)
				local status, msg = coroutine.resume(bus.input_co, bus, data)
				if not status then print('DCM | In err', msg) end
			elseif bus.read_reg and t_read > bus.read_to then
				-- Not ready and out of timeout, so force a termination
				print("DCM | TIMEOUT", bus.chain.name, bus.read_reg)
				bus.input_co = nil
				bus.read_reg = nil
			end
		end
		-- Process the read coroutines
		for bus_id, bus in ipairs(buses) do
			_co = bus.input_co
			if _co then
				running, pkt = coroutine.resume(_co)
				while pkt do
					if bus.read_reg=='position' then
						parse_read_position(pkt, bus.chain.is_mx)
					else
						parse_read_packet(pkt, bus.read_reg, bus.chain.is_mx)
					end
					if coroutine.status(_co)=='dead' then break end
					running, pkt = coroutine.resume(_co)
				end
			end
		end
		-- Debug
		t_end = get_time()
		if t_end - t_debug > 1 then
			local kb = collectgarbage('count')
			local debug_str = {
				string.format('\nDCM | Uptime %.2f sec, Mem: %d kB', t_end - t0, kb),
			}
			for bus_id, bus in ipairs(buses) do
				table.insert(debug_str, string.format(
					'%s Command Rate %.1f', bus.chain.name, bus.cmds_cnt / (t_end - bus.cmds_t0)
				))
				bus.cmds_cnt = 0
				bus.cmds_t0 = t_end
			end
			print(table.concat(debug_str, '\n'))
			t_debug = t_end
		end
		-- Steady timing
		collectgarbage('step')
		-- Sleep until it is the write cycle
		local t_sleep = WRITE_TIMEOUT - (t_read - t_write)
		if t_sleep>0 then usleep(t_sleep * 1e6) end
	end
end

local buses = {}
for chain_id, chain in ipairs(dcm_chains) do
	print("Opening", chain.name, chain.device)
	local bus = lD.new_bus(chain.device)
	table.insert(buses, bus)
	-- Point to the configuration
	bus.chain = chain
	name_to_chain[chain.name] = chain
	-- Initialize the bus
	entry(bus)
end

-- Begin the loop
loop(buses)
