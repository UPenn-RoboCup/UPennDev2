-- DCM is a thread
-- We are always a child process
local CTX, metadata = ...
-- Still need our library paths set
dofile'../include.lua'
-- Going to be threading this
local simple_ipc = require'simple_ipc'
-- Import the context
simple_ipc.import_context( CTX )
-- Communicate with the master body_wizard
local pair_ch = simple_ipc.new_pair(metadata.ch_name)

-- Set up the Dynamixel bus
local bus = libDynamixel.new_bus(metadata.device)
-- Grab the Joint IDs for this thread
local ids = metadata.ids
-- Corresponding Motor ids
local m_ids = metadata.m_ids
-- Verify that they are present
for id,m_id in pairs(ids) do
	assert(bus:send_ping(m_id),id..' not present.')
end
-- Access the typical commands quickly
local cp = jcm.writePtr.command_position
local cp_vals = {}
local cp_cmd = libDynamixel.set_nx_command_position
local p_read = libDynamixel.get_nx_position
-- Remove metadata and Config
Config, metadata = nil, nil
-- Begin infinite loop
while true do
	-- First, write position to the motors
	for i,id in ipairs(ids) do cp_vals[i] = cp[id] end
	-- Send on the bus
	local ret = cp_cmd(m_ids,cp_vals,bus)
	-- Next, read the positions
	local status = p_read(m_ids,bus)
	print("Got status",status)
end
