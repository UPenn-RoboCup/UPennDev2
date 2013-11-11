--------------------------------
-- Motion Communication Module
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local memory = require'memory'
local vector = require'vector'

-- shared properties
local shared = {}
local shsize = {}

-- Storing current stance info
shared.stance = {}
shared.stance.bodyTilt   = vector.zeros(1)
shared.stance.bodyHeight = vector.zeros(1)
shared.stance.bodyHeightTarget = vector.zeros(1)
shared.stance.uTorsoComp = vector.zeros(2) --For quasi-static balancing

--Arm info

shared.arm = {}
--Target arm position (w/o compensation)
shared.arm.qlarm = vector.zeros(7)
shared.arm.qrarm = vector.zeros(7)

--Torso-Compensated arm position
shared.arm.qlarmcomp = vector.zeros(7)
shared.arm.qrarmcomp = vector.zeros(7)



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

--Current Foot and Torso Poses
--TODO: extend to 6D poses
shared.status.uLeft = vector.zeros(3)
shared.status.uRight = vector.zeros(3)
shared.status.uTorso = vector.zeros(3)

--We store the torso velocity (to handle stopping)
shared.status.uTorsoVel = vector.zeros(3)

--ZMP is stored here for external monitoring
shared.status.uZMP = vector.zeros(3)

--If we are kneeling, we don't need quasistatic balancing
shared.status.iskneeling    = vector.zeros(1)

--If we are in wide stance, we don't use lateral quasistatic balancing 
shared.status.iswidestance    = vector.zeros(1)

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




local maxSteps = 40
shared.step = {}

--Footsteps queue
--{[rx ry ra supportLeg t0 t1 t2 zmpmodx zmpmody zmpmoda stepparam1 stepparam2 stepparam3]}
shared.step.footholds  = vector.zeros(13*maxSteps)
shared.step.nfootholds = vector.zeros(1)



memory.init_shm_segment(..., shared, shsize)