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

-- Combined pose
shared.robot.pose = vector.zeros(3)
-- Webot pose from GPS
shared.robot.gps = vector.zeros(3)
-- Accumulated Odometry
shared.robot.odometry = vector.zeros(3)
-- Walking Odometry
shared.robot.utorso0 = vector.zeros(3)
shared.robot.utorso1 = vector.zeros(3)

-- Get the map goal
shared.map = {}
shared.map.goal = vector.zeros(3)

-- Call the initializer
memory.init_shm_segment(..., shared, shsize)
