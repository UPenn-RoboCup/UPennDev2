--------------------------------
-- Motion Communication Module
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local memory = require'memory'

-- shared properties
local shared = {}
local shsize = {}

-- For the vision system
shared.camera = {}
shared.camera.bodyTilt   = vector.zeros(1)
shared.camera.bodyHeight = vector.zeros(1)

-- Walk Parameters
shared.walk = {}
shared.walk.bodyOffset = vector.zeros(3)
shared.walk.tStep      = vector.zeros(1)
shared.walk.bodyHeight = vector.zeros(1)
shared.walk.stepHeight = vector.zeros(1)
shared.walk.footY      = vector.zeros(1)
shared.walk.supportX   = vector.zeros(1)
shared.walk.supportY   = vector.zeros(1)
shared.walk.vel        = vector.zeros(3)
shared.walk.bipedal    = vector.zeros(1)
-- These should be status params
shared.walk.uLeft      = vector.zeros(3)
shared.walk.uRight     = vector.zeros(3)

-- Motion Status
shared.status = {}
shared.status.velocity = vector.zeros(3)
shared.status.odometry = vector.zeros(3)
shared.status.bodyOffset = vector.zeros(3)
shared.status.falling  = vector.zeros(1)

memory.init_shm_segment(..., shared, shsize)