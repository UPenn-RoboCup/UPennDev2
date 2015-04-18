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

-- Mesh formation
-- Sweep: Field of View (radians), time to complete sweep (seconds)
-- fov: In a single scan, which lidar ranges to use (field of view)
-- Dynamic range: min and max ranges to send
shared.mesh0 = {
	-- Chest
	sweep = {70 * DEG_TO_RAD, 3},
	fov = {-30*DEG_TO_RAD, 90*DEG_TO_RAD},
	dynrange = {0.1, 2}
}
shared.mesh1 = {
	-- Head
	sweep = {45 * DEG_TO_RAD, 2},
	fov = {-90*DEG_TO_RAD, 90*DEG_TO_RAD},
	dynrange = {0.15, 1.5}
}

-- Initialize the segment
memory.init_shm_segment(..., shared, shsize)
