local Body = {}

-- Useful constants
local DEG_TO_RAD = math.pi/180
local RAD_TO_DEG = 180/math.pi
Body.DEG_TO_RAD = DEG_TO_RAD
Body.RAD_TO_DEG = RAD_TO_DEG

-- Utilities
local unix         = require'unix'
local vector       = require'vector'
local quaternion   = require'quaternion'
local Transform    = require'Transform'
local util         = require'util'

-- Get time (for the real robot)
local get_time = unix.time

-- Shared memory
require'jcm'

-- Entry initializes the hardware of the robot
Body.entry = function()
  
end

-- Update speaks to the hardware of the robot
Body.update = function()
end

-- Exit gracefully shuts down the hardware of the robot
Body.exit = function()
end