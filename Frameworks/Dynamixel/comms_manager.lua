local libDynamixel = require('libDynamixel');
local unix = require 'unix'

-- SHM of the joint positions
--require "dcm"

local function entry()
end

local function update()
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