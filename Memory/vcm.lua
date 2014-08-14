---------------------------------
-- Vision Communication Module --
-- (c) 2013, 2014 Stephen McGill     --
---------------------------------
local memory = require'memory'
local vector = require'vector'
local shared = {}
local shsize = {}

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
shared.mesh.sweep = {math.pi/2, 5}
-- fov: In a single scan, which ranges to use (field of view)
shared.mesh.fov = {-math.pi/3, math.pi/3}
-- Net: {request, destination, compression}
-- request: 1 means mesh needs to be sent to destination
-- destination: 0 is udp, 1 is tcp PUB
-- compression: 0 is JPEG, 1 is PNG
-- is_streaming: 1 is LidarFSM setting mesh_net on each sweep
shared.mesh.net = {0, 0, 0, 0}
-- Net: {direction}
-- direction (set by LidarFSM): -1 left, 0 unknown, 1 right
shared.mesh.state = {0}
-- Dynamic range
shared.mesh.dynrange = {0.1, 1}

-- Initialize the segment
memory.init_shm_segment(..., shared, shsize)
