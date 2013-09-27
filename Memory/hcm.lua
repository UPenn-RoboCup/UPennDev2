--------------------------------
-- Human Communication Module --
-- (c) 2013 Stephen McGill    --
--------------------------------
local vector = require'vector'
local memory = require'memory'
local nJoints = 40
local maxWaypoints = 4

local shared_data = {}
local shared_data_sz = {}

-- Desired joint properties
shared_data.joints = {}
-- x,y,z,roll,pitch,yaw
shared_data.joints.plarm  = vector.zeros( 6 )
shared_data.joints.prarm  = vector.zeros( 6 )
-- TODO: 6->7 arm joint angles
shared_data.joints.qlarm  = vector.zeros( 6 )
shared_data.joints.qrarm  = vector.zeros( 6 )
-- 3 finger joint angles
shared_data.joints.qlgrip = vector.zeros( 3 )
shared_data.joints.qrgrip = vector.zeros( 3 )

shared_data.joints.qlshoulderyaw = vector.zeros( 1 )
shared_data.joints.qrshoulderyaw = vector.zeros( 1 )
-- Teleop mode
-- 1: joint, 2: IK
shared_data.joints.teleop = vector.ones( 1 )

-- Motion directives
shared_data.motion = {}
shared_data.motion.velocity = vector.zeros(3)
-- Emergency stop of motion
shared_data.motion.estop = vector.zeros(1)
-- Waypoints
-- {[x y a][x y a][x y a][x y a]...}
shared_data.motion.waypoints  = vector.zeros(3*maxWaypoints)
-- How many of the waypoints are actually used
shared_data.motion.nwaypoints = vector.zeros(1)
-- Local or global waypoint frame of reference
-- 0: local
-- 1: global
shared_data.motion.waypoint_frame = vector.zeros(1)
-- following mode
-- 1: simple, 2: robocup
shared_data.motion.follow_mode = vector.ones(1)


-------------------------------
-- Task specific information --
-------------------------------
-- Wheel/Valve
shared_data.wheel = {}
-- This has all values: the right way, since one rpc call
-- {handlepos(3) handleyaw handlepitch handleradius}
shared_data.wheel.model = vector.zeros(6)
shared_data.wheel.turnangle = vector.zeros(1)

-- Door Opening
shared_data.door = {}

--1 for left, 0 for right
shared_data.door.hand=vector.ones(1)

--Hinge XYZ pos from robot frame
shared_data.door.hinge_pos = vector.new({0.45,0.90,-0.15})
shared_data.door.hinge_pos = vector.new({0.45,0.85,-0.15})

--Radius of the door from hinge to the gripping position
--negavive value: left hinge, positive value: right hinge
shared_data.door.r = vector.new({-0.60})

--How much the handle sticks out from the door surface?
shared_data.door.grip_offset_x = vector.new({-0.05})

--The current angle of the door 
shared_data.door.yaw = vector.zeros(1)

--The target angle of the door 
shared_data.door.yaw_target = vector.new({-20*math.pi/180})


--[[
--Right hand door with right edge
shared_data.door.hand=vector.zeros(1)
shared_data.door.hinge_pos = vector.new({0.45,-0.90,-0.15})
shared_data.door.r = vector.new({0.60})
shared_data.door.yaw_target = vector.new({20*math.pi/180})
--]]




--[[
shared_data.door.hinge  = vector.new({.2,0,.1})
-- What is the position of the handle axis
-- What angle from the horizon is the handle
-- What is the length of the handle
-- {axis_x,axis_y,axis_z, angle, length}
shared_data.door.handle = vector.zeros( 6 )
-- How much to open the door
shared_data.door.open_ang   = vector.zeros(1)
-- Which hand opens the door?
-- 0: left
-- 1: right
shared_data.door.hand   = vector.zeros(1)
--]]

-- Dipoles for arbitrary grabbing
-- TODO: Use this in place of the wheel/door?
shared_data.left = {}
shared_data.left.cathode = vector.zeros(3)
shared_data.left.anode = vector.zeros(3)
-- strata (girth) / angle of attack / climb (a->c percentage)
shared_data.left.grip = vector.zeros(3)
----
shared_data.right = {}
shared_data.right.cathode = vector.zeros(3)
shared_data.right.anode = vector.zeros(3)
-- strata (girth) / angle of attack / climb (a->c percentage)
shared_data.right.grip = vector.zeros(3)

-- Call the initializer
memory.init_shm_segment(..., shared_data, shared_data_sz)
