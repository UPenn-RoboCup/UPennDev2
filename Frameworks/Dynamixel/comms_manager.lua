local libDynamixel = require('libDynamixel');
local unix = require 'unix'

-- SHM of the joint positions
local jcm = require'jcm'

local function entry()
end

-- Update the shared memory table
local function update()
	local res= Dynamixel:get_nx_position( jcm.left_nx_ids )
	local command_position = jcm.set_left_arm_position()
	local command_position = jcm.get_left_arm_command_position()
	local ret= Dynamixel:set_nx_command_position(jcm.left_nx_ids,command_position)
end

local function exit()
end

-- Just update the left arm
local cnt = 0
local t_start = unix.time()
local t_last = t_start
local t_debug = t_start
local t = t_last
entry()
while true do
	-- Timing
	t_last = t
	t = unix.time()
	local t_diff = t - t_last
	update()
	cnt = cnt+1
	-- Debugging
	-- Once a second
	local t_diff_debug = t - t_debug
	if t_diff_debug > 1 then
		print( string.format('DCM | Updating at %.2f FPS',cnt/t_diff_debug) )
		t_debug = t
		cnt = 0
	end
end