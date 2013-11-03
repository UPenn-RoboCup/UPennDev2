--------------------------------
-- Motion Communication Module
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local memory = require'memory'
local vector = require'vector'

-- shared properties
local shared = {}
local shsize = {}

-- For the vision system 
-- Now FK / IK use this too! 

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
shared.walk.stoprequest= vector.zeros(1)
shared.walk.ismoving= vector.zeros(1)

-- Walk-step transition
shared.walk.steprequest= vector.zeros(1)
shared.walk.step_supportleg= vector.zeros(1)


-- Motion Status
shared.status = {}
shared.status.velocity   = vector.zeros(3)
shared.status.odometry   = vector.zeros(3)
shared.status.bodyOffset = vector.zeros(3)
shared.status.falling    = vector.zeros(1)

shared.status.bodyHeight = vector.zeros(1) --for sit/standup
--Current Foot and Torso Poses
--TODO: extend to 6D poses
shared.status.uLeft = vector.zeros(3)
shared.status.uRight = vector.zeros(3)
shared.status.uTorso = vector.zeros(3)

--We store the torso velocity (to handle stopping)
shared.status.uTorsoVel = vector.zeros(3)

--ZMP is stored here for external monitoring
shared.status.uZMP = vector.zeros(3)

--Current time
shared.status.t = vector.zeros(1)

-- Foot support
--SJ: they are bit misleading as they are different from 'support' positions
shared.support = {}
shared.support.uLeft_now   = vector.zeros(3)
shared.support.uRight_now  = vector.zeros(3)
shared.support.uTorso_now  = vector.zeros(3)
shared.support.uLeft_next  = vector.zeros(3)
shared.support.uRight_next = vector.zeros(3)
shared.support.uTorso_next = vector.zeros(3)




memory.init_shm_segment(..., shared, shsize)