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
-- Roll, pitch, yaw of the wire
shared.wire.rpy = vector.zeros(3)
-- Roll and Yaw in the camera frame
shared.wire.cam_ry = vector.zeros(2)


------------------------
-- Initialize the segment
memory.init_shm_segment(..., shared, shsize)
