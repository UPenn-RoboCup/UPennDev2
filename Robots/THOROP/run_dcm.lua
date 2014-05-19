-- DCM is a thread or standalone
local CTX, metadata = ...
-- Still need our library paths set
dofile'include.lua'
assert(ffi, 'DCM | Please use LuaJIT :). Lua support in the near future')
-- Going to be threading this
local si = require'simple_ipc'
-- Import the context
--print('CTX',CTX,type(CTX))
local parent_ch, IS_THREAD
if CTX and not arg then
	IS_THREAD = true
	si.import_context(CTX)
	-- Communicate with the master thread
	parent_ch = si.new_pair(metadata.ch_name)
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
-- Debug
if metadata.name then print('DCM | Running', metadata.name) end
-- Modules
require'jcm'
local lD = require'libDynamixel'
local ptable = require'util'.ptable
local usleep, get_time = unix.usleep, unix.time
-- Corresponding Motor ids
local bus = lD.new_bus(metadata.device)
local m_ids = metadata.m_ids
if not m_ids then
	m_ids = bus:ping_probe()
	print('DCM | FOUND', unpack(m_ids))
end
local n_motors = #m_ids
-- Verify that the m_ids are present
for _,m_id in pairs(m_ids) do
	print('PING', m_id)
	local p = bus:ping(m_id)
	assert(p[1], string.format('DCM | ID %d not present.', m_id))
	--ptable(p[1])
	usleep(1e3)
end
-- Cache some settings from the Config
local min_rad, max_rad, is_unclamped = Config.servo.min_rad, Config.servo.max_rad, Config.servo.is_unclamped
local direction, to_steps, step_zero = Config.servo.direction, Config.servo.to_steps, Config.servo.step_offset
local to_radians, step_offset, m_to_j = Config.servo.to_radians, Config.servo.step_zero, Config.servo.motor_to_joint
-- Make the reverse mapping for our motor ids of this thread
local j_ids = {}
for i,m_id in ipairs(m_ids) do j_ids[i] = m_to_j[m_id] end
-- Clamp the radian within the min and max
-- Used when sending packets and working with actuator commands
local function radian_clamp (idx, radian)
	if is_unclampled[idx] then return radian end
	return math.min(math.max(radian, min_rad[idx]), max_rad[idx])
end
-- Radian to step, using offsets and biases
local function radian_to_step (idx, radian)
	return math.floor(direction[idx] * radian_clamp(idx, radian) * to_steps[idx] + step_zero[idx] + step_offset[idx])
end
-- Step to radian
local function step_to_radian (idx, step)
	return direction[idx] * to_radians[idx] * (step - step_zero[idx] - step_offset[idx])
end
-- Cache the typical commands quickly
local cp_ptr  = jcm.actuatorPtr.command_position
local cp_cmd  = lD.set_nx_command_position
local p_ptr   = jcm.sensorPtr.position
local p_read  = lD.get_nx_position
local p_parse = lD.byte_to_number[lD.nx_registers.position[2]]
-- Define reading
local positions = vector.zeros(n_motors)
local function do_read (is_strict)
	local status = p_read(m_ids, bus)
	if is_strict and #status~=n_motors then return end
	for _,s in ipairs(status) do
		local p = p_parse(unpack(s.parameter))
		local j_id = m_to_j[s.id]
		local r = step_to_radian(j_id, p)
		-- Set in SHM
		-- TODO: Worry about the cache charing among threads
		p_ptr[j_id-1] = r
		-- Keep a local copy
		-- TODO: Send this copy back to the master thread?
		positions[j_id] = r
	end
	return true
end
-- Define writing
-- TODO: Add MX support
local commands = vector.zeros(n_motors)
local function do_write ()
	for _,j_id in ipairs(j_ids) do
		commands[i] = radian_to_step(cp_ptr[j_id-1])
	end
	-- Perform the sync write
	cp_cmd(m_ids, commands, bus)
end
-- Define parent interaction. NOTE: Openly subscribing to ANYONE. fiddle even
local function process_parent (msg)

end
-- Initially, copy the command positions from the read positions
-- Try 5 times to get all joints at once
local did_read_all
for i=1,5 do
	did_read_all = do_read(true)
	if did_read_all then
		-- FFI is 0 indexed
		for _,j_id in ipairs(j_ids) do cp_ptr[j_id-1] = p_ptr[j_id-1] end
	end
end
assert(did_read_all, 'Did not initialize the motors properly!')
-- Collect garbage before starting
collectgarbage()
-- Begin infinite loop
local t0 = get_time()
local t_debug = t0
while true do
	local t = get_time()
	local t_diff = t-t0
	t0 = t
	--------------------
	-- Periodic Debug --
	--------------------
  if t - t_debug>1 then
	  print('\nDCM | t_diff', t_diff, 1 / t_diff)
    ptable(positions)
    t_debug = t
  end
	--------------------
	-- Read Positions --
	--------------------
	do_read()
	---------------------
	-- Write Positions --
	---------------------
	do_write()
	---------------------
	-- Parent Commands --
	---------------------
	local parent_msg = parent_ch:receive(true)
	if parent_msg then
		if parent_msg=='exit' then
			bus:close()
			if IS_THREAD then parent_msg:send'done' end
			return
		else
			process_parent(parent_msg)
		end
	end
end
