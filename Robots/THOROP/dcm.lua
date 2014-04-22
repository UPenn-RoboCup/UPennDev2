-- DCM is a thread
-- We are always a child process
local CTX, metadata = ...
-- Still need our library paths set
dofile'include.lua'
-- Going to be threading this
local simple_ipc = require'simple_ipc'
-- Import the context
local pair_ch
if CTX then
	simple_ipc.import_context( CTX )
	-- Communicate with the master body_wizard
	pair_ch = simple_ipc.new_pair(metadata.ch_name)
else
	metadata = {
		device = '/dev/cu.usbserial-FTVTLUY0B',
		m_ids = {2,4,6,8},
	}
end
-- Set up the Dynamixel bus
local libDynamixel = require'libDynamixel'
local bus = libDynamixel.new_bus(metadata.device)
-- Corresponding Motor ids
local m_ids = metadata.m_ids
local n_motors = #m_ids
metadata = nil
-- Grab the Joint IDs for this thread
-- NOTE: Joint IDs start at 0 if ffi
-- Joint IDs start at 1 if carray
local Body = require'Body'
local m_to_j = Body.servo.motor_to_joint
local step_to_radian = Body.make_joint_radian
Body = nil
local j_ids = {}
for i,m_id in ipairs(m_ids) do
  j_ids[i] = m_to_j[m_id]
end
-- Verify that they are present
for id,m_id in pairs(m_ids) do
	assert(bus:ping(m_id),string.format('ID %d not present.',m_id))
	unix.usleep(1e3)
end
-- Access the typical commands quickly
require'jcm'
local cp_ptr = jcm.writePtr.command_position
local cp_cmd = libDynamixel.set_nx_command_position
local cp_vals = {}
local p_ptr = jcm.writePtr.command_position
local p_read = libDynamixel.get_nx_position
local p_parse = libDynamixel.byte_to_number[libDynamixel.nx_registers.position[2]]
-- Remove Config
Config = nil
-- Cannot nil the jcm...
-- Let's think about this a bit more, then...
-- May wish to have a smarter memory system?
--jcm = nil

-- Initial position reading
local status = p_read(m_ids,bus)
for _,s in ipairs(status) do
	local p = p_parse(unpack(s.parameter))
	local j_id = m_to_j[s.id]
	local r = step_to_radian(j_id,p)
	p_ptr[j_id-1] = r
end
-- TODO: Use ffi.copy
local a, b = 1, n_motors
if ffi then
	a = a - 1
	b = b - 1
end
-- Copy into the commands
for i=a,b do cp_ptr[i] = p_ptr[i] end

-- Begin infinite loop
local t0 = unix.time()
while true do
	local t = unix.time()
	local t_diff = t-t0
	t0 = t
	if t_diff>0.02 then
		print('Slow dcm FPS',1/t_diff,t_diff)
	end
	-- First, write position to the motors
	for i,j_id in ipairs(j_ids) do
		if ffi then j_id = j_id - 1 end
		cp_vals[i] = cp_ptr[j_id]
	end
	-- Send on the bus
	--local ret = cp_cmd(m_ids,cp_vals,bus)
	-- Next, read the positions
	local status = p_read(m_ids,bus)
	if type(status)=='table' then
		for _,s in ipairs(status) do
			local p = p_parse(unpack(s.parameter))
			local j_id = m_to_j[s.id]
			local r = step_to_radian(j_id,p)
			if ffi then j_id = j_id - 1 end
			p_ptr[j_id] = r
		end
	end
	-- Set the data in shared memory
	-- TODO: Check if any messages from the master thread
end
