---------------------------------
-- Vision Communication Module --
-- (c) 2013, 2014 Stephen McGill     --
---------------------------------
local memory = require'memory'
local vector = require'vector'
local shared = {}
local shsize = {}


-- RRT
shared.RRT = {}
shared.RRT.finished = vector.zeros(1)

-- Mesh formation
-- Sweep: Field of View (radians), time to complete sweep (seconds)
-- fov: In a single scan, which lidar ranges to use (field of view)
-- Dynamic range: min and max ranges to send

shared.rrt1 = {
	finished = vector.zeros(1)
}

-- Initialize the segment
memory.init_shm_segment(..., shared, shsize)
