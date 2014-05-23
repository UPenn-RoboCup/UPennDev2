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
-- Model: Radius and Distance to the wire, from the camera, last model time
shared.wire.model = vector.zeros(3)
-- Bounding box of where we are looking currently
shared.wire.bbox = vector.zeros(4)

------------------------
-- Initialize the segment
memory.init_shm_segment(..., shared, shsize)
