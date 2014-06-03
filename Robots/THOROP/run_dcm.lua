-- DCM is a thread or standalone
local CTX, metadata = ...
-- Still need our library paths set
dofile'include.lua'
assert(ffi, 'DCM | Please use LuaJIT :). Lua support in the near future')
-- Going to be threading this
local si = require'simple_ipc'
local vector = require'vector'
-- Import the context
--print('CTX',CTX,type(CTX))
local parent_ch, IS_THREAD
if CTX and not arg then
	IS_THREAD = true
	si.import_context(CTX)
	-- Communicate with the master thread
	parent_ch = si.new_pair(metadata.ch_name)
  parent_ch:send'init'
else
	-- Set metadata based on command line arguments
	local chain_id, chain = tonumber(arg[1])
	if chain_id then
		metadata = Config.chain[chain_id]
		-- Make reverse subscriber for the chain
		parent_ch = si.new_subscriber('dcm'..chain_id..'!')
	else
		-- Make reverse subscriber for the anonymous chain
		parent_ch = si.new_subscriber('dcm!')
	end
end
-- Fallback on undefined metadata
metadata = metadata or {}
local DISABLE_READ = false
local debug_prefix = 'DCM '..metadata.name..' |'
-- Debug
print(debug_prefix, 'running')
-- Modules
require'dcm'
local lD = require'libDynamixel'
local ptable = require'util'.ptable
local usleep, get_time = unix.usleep, unix.time
local running = true
-- Corresponding Motor ids
local bus = lD.new_bus(metadata.device)
local m_ids = metadata.m_ids
if not m_ids then
	m_ids = bus:ping_probe()
	print(debug_prefix, 'FOUND', unpack(m_ids))
end
local n_motors = #m_ids
-- Verify that the m_ids are present
for _,m_id in pairs(m_ids) do
	--print('PING', m_id)
	local p = bus:ping(m_id)
	assert(p[1], string.format('%s ID %d not present.', debug_prefix, m_id))
	-- Check the status return level
	local s = lD.get_nx_return_delay_time(m_id, bus)
	local rdt = lD.byte_to_number[lD.nx_registers.return_delay_time[2]](unpack(s[1].parameter))
	if rdt~=0 then
		usleep(5e3)
		lD.set_nx_return_delay_time(m_id, 0, bus)
	end
	print('Return delay', rdt, m_id)
	usleep(5e3)
end
-- Cache some settings from the Config
local min_rad, max_rad, is_unclamped = Config.servo.min_rad, Config.servo.max_rad, Config.servo.is_unclamped
local direction, to_steps, step_zero = Config.servo.direction, Config.servo.to_steps, Config.servo.step_offset
local to_radians, step_offset, m_to_j = Config.servo.to_radians, Config.servo.step_zero, Config.servo.motor_to_joint
-- Make the reverse mapping for our motor ids of this thread
local j_ids, m_to_order, j_to_order = {}, {}, {}
for i,m_id in ipairs(m_ids) do
  local j_id = m_to_j[m_id]
  j_ids[i] = j_id
  m_to_order[m_id] = i
  j_to_order[j_id] = i
end
-- Clamp the radian within the min and max
-- Used when sending packets and working with actuator commands
local function radian_clamp (idx, radian)
	if is_unclamped[idx] then return radian end
	return math.min(math.max(radian, min_rad[idx]), max_rad[idx])
end
-- Radian to step, using offsets and biases
local function radian_to_step (idx, radian)
	return math.floor(direction[idx] * radian_clamp(idx, radian) * to_steps[idx] + step_zero[idx] + step_offset[idx])
end
-- Step to radian
local function step_to_radian (idx, step)
  assert(idx, debug_prefix..'NO MOTOR IDX')
  assert(step, debug_prefix..'NO STEP '..idx)
	return direction[idx] * to_radians[idx] * (step - step_zero[idx] - step_offset[idx])
end
-- Cache the typical commands quickly
local cp_ptr  = dcm.actuatorPtr.command_position
local tq_ptr  = dcm.actuatorPtr.torque_enable
local cp_cmd  = lD.set_nx_command_position
local tq_cmd  = lD.set_nx_torque_enable
local p_ptr   = dcm.sensorPtr.position
local p_read  = lD.get_nx_position
local p_parse = lD.byte_to_number[lD.nx_registers.position[2]]
local tq_read  = lD.get_nx_torque_enable
local tq_parse = lD.byte_to_number[lD.nx_registers.torque_enable[2]]
-- Define reading
local tq_enable = vector.zeros(n_motors)
local positions = vector.zeros(n_motors)
local read_status, read_pkt, read_j_id
local function do_read()
	if DISABLE_READ then
		for i,j_id in ipairs(j_ids) do
			p_ptr[j_id-1] = cp_ptr[j_id-1]
		end
	end
	-- TODO: Strict should just be one motor at a time...
	read_status = p_read(m_ids, bus)
  local rad
	for _,s in ipairs(read_status) do
    if s.error>0 then
      print(debug_prefix, 'READ ERROR', s.error, s.id)
    else
		  read_pkt, read_j_id = p_parse(unpack(s.parameter)), m_to_j[s.id]
  		rad = step_to_radian(read_j_id, read_pkt)
	  	-- Set in SHM
		  -- TODO: Worry about the cache charing among threads
  		p_ptr[read_j_id-1] = rad
	  	-- Keep a local copy
		  -- TODO: Send this copy back to the master thread?
  		positions[j_to_order[read_j_id]] = rad
    end
	end
	--usleep(1e3)
end
-- Define writing
-- TODO: Add MX support
local commands = vector.zeros(n_motors)
local function do_write()
	for i,j_id in ipairs(j_ids) do
		commands[i] = radian_to_step(j_id, cp_ptr[j_id-1]) or commands[i]
	end
  --error(string.format('%s | %s: %s', metadata.name, table.concat(m_ids, ' '), tostring(commands)) )
	-- Perform the sync write
	cp_cmd(m_ids, commands, bus)
	--usleep(2e3)
end
-- Define parent interaction. NOTE: Openly subscribing to ANYONE. fiddle even
local parent_cb = {
	exit = function()
		running = false
	end,
  torque_enable = function()
    local j_id, status, val
    local valid_tries = 0
    for _, m_id in ipairs(m_ids) do
      j_id = m_to_j[m_id] - 1
      val = tq_ptr[j_id]
			-- Safely copy over the position to command_position
			cp_ptr[j_id] = p_ptr[j_id]
      while not valid do
        status = tq_cmd(m_id, val, bus)
        if status[1] and status[1].error==0 then break end
        valid_tries = valid_tries + 1
        assert(valid_tries<5, debug_prefix..' CANNOT SET TORQUE! Motor_id '..m_id)
      end
      tq_enable[m_to_order[m_id]] = val
    end
  end
}

-- Check the F/T based on the name of the chain
-- Declare the struct
ffi.cdef[[ typedef struct { int16_t a, b, c, d; } ft; ]]
if metadata.name=='lleg' then
	parent_cb.ft = function ()
		local ptr, read = dcm.actuatorPtr[cmd], lD.get_nx_data
		local status = read({24, 26}, bus)
		-- Should be 4 16-bit integers, so 8 bytes
		local l_ft = ffi.cast('ft*', ffi.new('int8_t[8]', status[1].parameter))
		-- Just do a sync read here; can try reliable later, if desired...
		print('DCM | ft', 3.3 * l_ft.a / 4096 - 1.65)
    print(l_ft.a,l_ft.b,l_ft.c,l_ft.d)
	end
elseif metadata.name=='rleg' then

end

local function do_parent (p_skt)
  local cmds = p_skt:recv_all()
	if not cmds then return end
	for i, cmd in ipairs(cmds) do
		-- Check if there is something special
		local f = parent_cb[cmd]
		if f then return f() end
		-- Else, access something from the motor
		local ptr, set = dcm.actuatorPtr[cmd], lD['set_nx_'..cmd]
		if not set then return end
		-- TODO: Check if we need the motors torqued off for the command to work
		-- Send individually to the motors, waiting for the status return
		local j_id, status
		for _, m_id in ipairs(m_ids) do
			j_id = m_to_j[m_id] - 1
			status = set(m_id, ptr[j_id], bus)
			-- TODO: check the status, and repeat if necessary...
		end
		usleep(1e3)
	end
end
-- Initially, copy the command positions from the read positions
for _, m_id in ipairs(m_ids) do
	local status, s = {}
	while not s do status = p_read(m_id, bus); s = status[1] end
	if not s.parameter then
		print(debug_prefix, 'ERROR initial', s.error, s.id)
	else
		local p = p_parse(unpack(s.parameter))
		local j_id = m_to_j[s.id]
		local r = step_to_radian(j_id, p)
		p_ptr[j_id-1] = r
		cp_ptr[j_id-1] = r
		local idx = j_to_order[j_id]
		positions[idx] = r
		commands[idx] = r
		-- Read the current torque states
		s = nil
		while not s do status = tq_read(m_id, bus); s = status[1] end
		local tq_en = tq_parse(unpack(s.parameter))
		j_id = m_to_j[s.id]
		idx = j_to_order[j_id]
		tq_enable[idx] = tq_en
	end
end

parent_ch.callback = do_parent
local poller = si.wait_on_channels{parent_ch}

-- Begin infinite loop
print(debug_prefix, 'Begin loop')
local t0, t = get_time()
local t_debug, t_last, t_diff = t0, t0
local t_sleep = 1/125
while running do
	t_last = t
	t = get_time()
	--------------------
	-- Periodic Debug --
	--------------------
  if t - t_debug>1 then
    print(debug_prefix)
    --ptable(positions)
    t_debug = t
  end
	-- If no parent commands, then perform a read/write cycle
	if poller:poll(0)==0 then
		---------------------
		-- Write Positions --
		---------------------
		do_write()
		--------------------
		-- Read Positions --
		--------------------
		do_read()
	end
  -- Keep stable timing
  collectgarbage('step')
	--t_diff = get_time() - t
	--unix.usleep(1e6*(t_sleep-t_diff))
	usleep(1e6*t_sleep)
end

-- Exiting
print(debug_prefix, "Cleaning up")
bus:close()
if IS_THREAD then parent_ch:send'done' end
