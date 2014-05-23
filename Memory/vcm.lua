---------------------------------
-- Vision Communication Module --
-- (c) 2013 Stephen McGill     --
---------------------------------
local memory = require'memory'
local vector = require'vector'

-- shared properties
local shared = {}
local shsize = {}
-----------------
-- Head Camera --
shared.wire = {}
-- Last time the wire was seen
shared.wire.t = vector.zeros(1)
-- Roll, Pitch Yaw in the camera frame
shared.wire.cam_rpy = vector.zeros(3)
-- Mode: Radius and Distance to the wire, from the camera, last model time
shared.wire.model = vector.zeros(3)

------------------------
-- Initialize the segment
memory.init_shm_segment(..., shared, shsize)
