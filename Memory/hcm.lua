--------------------------------
-- Human Communication Module
-- (c) 2013 Stephen McGill
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

--------------------------------
-- Task specific information
--------------------------------
-- Wheel/Valve
shared_data.wheel = {}
-- This has all values: the right way, since one rpc call
-- {handlepos(3) handleyaw handlepitch handleradius}
shared_data.wheel.model = vector.zeros(6)
shared_data.wheel.turnangle = vector.zeros(1)

-- Door Opening
shared_data.door = {}
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

-- Call the initializer
memory.init_shm_segment(..., shared_data, shared_data_sz)