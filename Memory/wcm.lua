--------------------------------
-- World Communication Module
-- (c) 2013 Stephen McGill
--------------------------------
local vector = require'vector'
local memory = require'memory'
-- shared properties
local shared = {}
local shsize = {}

shared.robot = {}
-- Initial Pose
shared.robot.initialpose = vector.zeros(3)
-- Combined pose
shared.robot.pose = vector.zeros(3)
-- Webot pose from GPS
shared.robot.gps = vector.zeros(3)
-- (Full) Accumulated Odometry
shared.robot.odometry = vector.zeros(3)
-- Walking Odometry
shared.robot.utorso0 = vector.zeros(3)
shared.robot.utorso1 = vector.zeros(3)
shared.robot.t = vector.zeros(1)

-- Get the map goal
shared.map = {}
shared.map.goal = vector.zeros(3)
shared.map.waypoint = vector.zeros(3)
shared.map.enable_slam = vector.zeros(1)

-- Picking up the ball
-- These are global coordinates
shared.ball = {}
shared.ball.pose = vector.zeros(3)
shared.ball.t = vector.zeros(1)
shared.ball.pos = vector.zeros(3)
shared.ball.rot = vector.new({1,0,0,0,1,0,0,0,1})

-- Call the initializer
memory.init_shm_segment(..., shared, shsize)
