---------------------------------
-- Vision Communication Module --
-- (c) 2013, 2014 Stephen McGill     --
---------------------------------
local memory = require'memory'
local vector = require'vector'
local shared = {}
local shsize = {}

-- DRC network
shared.network = {}
-- Time of last observed good conditions
shared.network.tgood = vector.zeros(1)

-- RoboCup Ball
shared.ball = {}
shared.ball.centroid = vector.zeros(2)
shared.ball.diameter = vector.zeros(1)
shared.ball.detect = vector.zeros(1)
shared.ball.v = vector.zeros(3)
shared.ball.r = vector.zeros(1)
shared.ball.t = vector.zeros(1)

-- RoboCup Goal
shared.goal = {}
shared.goal.detect = vector.zeros(1)
shared.goal.enable = vector.zeros(1)
shared.goal.type = vector.zeros(1)
shared.goal.v1 = vector.zeros(4)
shared.goal.v2 = vector.zeros(4)
shared.goal.t = vector.zeros(1)

-- Ultrasound Sensor
shared.us = {}
shared.us.left      = vector.zeros(10)
shared.us.right     = vector.zeros(10)
shared.us.obstacles = vector.zeros(2)
shared.us.free      = vector.zeros(2)
shared.us.dSum      = vector.zeros(2)
shared.us.distance  = vector.zeros(2)

-- Mesh formation
shared.mesh = {}
-- Sweep: Field of View (radians), time to complete sweep (seconds)
shared.mesh.sweep = {60 * DEG_TO_RAD, 3}
-- fov: In a single scan, which ranges to use (field of view)
-- Like pitch: positive is down, negative is up. 90 deg is straight down
shared.mesh.fov = {0, math.pi/2}
-- Net: {request, destination, compression}
-- request: 1 means mesh needs to be sent to destination
-- compression: 0 is JPEG, 1 is PNG, 2 is RAW
-- is_streaming: 1 is LidarFSM setting mesh_net on each sweep
shared.mesh.net = {0, 1, 1}
-- Net: {direction}
-- direction (set by LidarFSM): -1 left, 0 unknown, 1 right
shared.mesh.state = {0}
-- Dynamic range
shared.mesh.dynrange = {0.1, 2}

-- Initialize the segment
memory.init_shm_segment(..., shared, shsize)
