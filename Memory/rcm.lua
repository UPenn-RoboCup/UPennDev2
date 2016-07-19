---------------------------------
-- RRT Communication Module --
-- (c) 2016 Jinwook Huh     --
---------------------------------
local memory = require'memory'
local vector = require'vector'
local shared = {}
local shsize = {}


-- RRT
shared.RRT = {}
shared.RRT.finished = vector.zeros(1)
shared.RRT.eef = vector.zeros(6)

-- Mesh formation
-- Sweep: Field of View (radians), time to complete sweep (seconds)
-- fov: In a single scan, which lidar ranges to use (field of view)
-- Dynamic range: min and max ranges to send

shared.rrt1 = {
	finished = vector.zeros(1)
--	eef = vector.zeros(6)
}

-- Initialize the segment
memory.init_shm_segment(..., shared, shsize)
