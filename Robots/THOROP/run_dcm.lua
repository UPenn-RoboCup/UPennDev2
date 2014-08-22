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
local WRITE_TIMEOUT = 1 / 200
local READ_TIMEOUT = 1 / 200

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
local left_ft, right_ft = Config.left_ft, Config.right_ft

-- DCM Cache
local cp_ptr = dcm.actuatorPtr.command_position
local tq_ptr = dcm.actuatorPtr.torque_enable
local p_ptr  = dcm.sensorPtr.position
local p_ptr_t = dcm.tsensorPtr.position

--------------------------
-- Global tmp variables --
--------------------------
local t_end, t_write, t_read, t_start
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

-- Standard Lua Cache
local min, max, floor = math.min, math.max, math.floor
local char = string.char
local sel, uread, get_time, usleep = unix.select, unix.read, unix.time, unix.usleep
local insert = table.insert

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

-- Position Packet
local function parse_read_position(pkt, bus)
	-- Nothing to do if an error
	if pkt.error ~= 0 then return end
	-- Assume just reading position, for now
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

-- General Read Packet
local function parse_read_packet(pkt, bus)
	if pkt.error ~= 0 then return end
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
	return read_j_id, j_val
end

-- Packet parsing lookup
local parse = setmetatable({}, {
  __index = function(t, k)
    return parse_read_packet
  end
})
parse.position = parse_read_position

-- Be able to form the read loop command on-demand
-- For now, just have the position
local function form_read_loop_cmd(bus, cmd)
	local rd_addrs, has_mx, has_nx = {}, false, false
	for _, m_id in ipairs(bus.m_ids) do
		local is_mx, is_nx = bus.has_mx_id[m_id], bus.has_nx_id[m_id]
		if is_mx then
			insert(rd_addrs, lD.mx_registers.position)
			has_mx = true
		else
			insert(rd_addrs, lD.nx_registers.position)
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

-- Parent command handling
local function do_external(request, bus)
	-- Check if writing to the motors
	local wr_reg, rd_reg = request.wr_reg, request.rd_reg
	local bus_name = request.bus
	local m_id
	if wr_reg then
		local ptr = dcm.actuatorPtr[wr_reg]
		local m_ids, m_vals, addr_n_len = {}, {}, {}
		local has_nx, has_mx = false, false
		for j_id, is_changed in pairs(request.ids) do
			m_id = j_to_m[j_id]
			if bus.has_mx_id[m_id] then
				has_mx = true
				insert(m_ids, m_id)
				insert(m_vals, ptr[j_id-1])
				insert(addr_n_len, lD.mx_registers[wr_reg])
			elseif bus.has_nx_id[m_id] then
				has_nx = true
				insert(m_ids, m_id)
				insert(m_vals, ptr[j_id-1])
				insert(addr_n_len, lD.nx_registers[wr_reg])
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
        if status and status.error==0 then
          -- Set the CP and the P
          if tq_val==1 then
            if is_mx then
              status = lD.get_mx_position(m_id, bus)[1]
            else
              status = lD.get_nx_position(m_id, bus)[1]
            end
            j_id, pos = parse_read_position(status, bus)
            if not j_id then print("Bad pos read in tq_en") end
            cp_ptr[j_id - 1] = p_ptr[j_id - 1]
          end
        else
          print("BAD TORQUE ENABLE", m_id, tq_val, status and status.error)
        end
      end
      -- Done the cycle if setting torque
      return
    end
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
				insert(m_ids, m_id)
				insert(addr_n_len, lD.mx_registers[rd_reg])
			elseif bus.has_nx_id[m_id] then
				has_nx = true
				insert(m_ids, m_id)
				insert(addr_n_len, lD.nx_registers[rd_reg])
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
	elseif request.ft then
    print("FORCE/TORQUE READING")
		if bus.name=='lleg' then
			local ft_reading = get_ft({24, 26}, left_ft, bus)
			if not ft_reading then return end
			-- Copy into SHM
			ffi.copy(dcm.sensorPtr.lfoot, ft_reading, ffi.sizeof(ft_reading))
		elseif bus.name=='rleg' then
			local ft_reading = get_ft({23, 25}, right_ft, bus)
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
			-- Only add position commands if torque enabled
			if tq_ptr[j_id-1]==1 then
				insert(send_ids, m_id)
				insert(commands, radian_to_step(j_id, cp_ptr[j_id-1]))
				insert(cmd_addrs,
					is_mx and lD.mx_registers.command_position or lD.nx_registers.command_position
				)
			end
		end
		-- Perform the sync write
		if #commands>0 then
			if has_nx and has_mx then
				lD.set_bulk(char(unpack(send_ids)), cmd_addrs, commands, bus)
			elseif has_nx then
				lD.set_nx_command_position(send_ids, commands, bus)
			else
				lD.set_bulk(char(unpack(send_ids)), cmd_addrs, commands, bus)
--				lD.set_nx_command_position(send_ids, commands, bus)
			end
		end
    -- One command gets a status return
		coroutine.yield(#commands==1 and 1 or 0)
		-- Run the parent queue until a write to the bus
		request = table.remove(bus.request_queue, 1)
		while request do
			n, reg = do_external(request, bus)
			if n then
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
			insert(queue, req)
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
	-- Add the fd
	insert(_fds, bus.fd)
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
      if status and status.error==0 then break end
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
      if status and status.error==0 then break end
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
		dcm.actuatorPtr.torque_enable[j_id-1] = tq_parse(unpack(status.parameter))
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
  --insert(numbered_buses, bus)
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
      if bus.npkt_to_expect == 0 then
    		bus.cmds_cnt = bus.cmds_cnt + 1
      else
    		bus.reads_cnt = bus.reads_cnt + bus.npkt_to_expect
      end
    end
	end
	-- Load data from the bus into the input coroutine
	sel_wait = WRITE_TIMEOUT
  do_collect = true
  -- Loop until the write timeout is expired
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
		t_end = get_time()
		sel_wait = WRITE_TIMEOUT - (t_end - t_start)
  end
	-- Debug messages for the user
  dt_debug = t_start - t_debug
	if dt_debug > 2 then
    t_debug = t_start
		debug_str = {
			string.format('\nDCM | Uptime %.2f sec, Mem: %d kB', t_start - t0, collectgarbage('count')),
		}
		for bname, bus in pairs(named_buses) do
			insert(debug_str, string.format(
				'%s Command Rate %.1f Hz with %d timeouts of %d reads', bname, bus.cmds_cnt / dt_debug, bus.n_read_timeouts, bus.reads_cnt)
			)
      bus.reads_cnt = 0
			bus.cmds_cnt = 0
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
