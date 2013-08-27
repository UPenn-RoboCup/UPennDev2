--------------------------------
-- Human Communication Module
-- (c) 2013 Stephen McGill
--------------------------------
local vector = require'vector'
local memory = require'memory'
local nJoints = 40

local shared_data = {}
local shared_data_sz = {}

-- Desired joint properties
shared_data.joints = {}
shared_data.joints.plarm  = vector.zeros( 6 ) -- x,y,z,roll,pitch,yaw
shared_data.joints.prarm  = vector.zeros( 6 )
shared_data.joints.qlarm  = vector.zeros( 6 ) -- 6 joints
shared_data.joints.qrarm  = vector.zeros( 6 )
shared_data.joints.qlgrip = vector.zeros( 3 ) -- 3 fingers
shared_data.joints.qrgrip = vector.zeros( 3 )

-- Motion directives
shared_data.motion = {}
shared_data.motion.velocity = vector.zeros(3)
-- Emergency stop of motion
shared_data.motion.estop = vector.zeros(1)

--------------------------------
-- Task specific information
--------------------------------
shared_data.wheel = {}
shared_data.wheel.pos        = vector.new({.3,0,.1})
shared_data.wheel.radius     = vector.new({.15})
shared_data.wheel.yawangle   = vector.zeros(1)
shared_data.wheel.pitchangle = vector.zeros(1)
shared_data.wheel.turnangle  = vector.zeros(1)

-- Call the initializer
memory.init_shm_segment(..., shared_data, shared_data_sz)