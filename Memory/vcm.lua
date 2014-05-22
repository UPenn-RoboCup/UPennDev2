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
-- Roll, pitch, yaw of the wire in absolute frame
shared.wire.rpy = vector.zeros(3)
-- Roll, Pitch Yaw in the camera frame
shared.wire.cam_rpy = vector.zeros(3)


------------------------
-- Initialize the segment
memory.init_shm_segment(..., shared, shsize)
