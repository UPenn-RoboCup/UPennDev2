--------------------------------
-- Motion Communication Module
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local memory = require'memory'
local vector = require'vector'

-- shared properties
local shared = {}
local shsize = {}


--Now we use shm variable here to toggle dcm leg reading (which is needed only for init)
shared.servo={}
shared.servo.read=vector.zeros(1)

--Now we put debug messages to DCM first
--So that we can show them in synchronous way
shared.servo.larm = vector.zeros(3)
shared.servo.rarm = vector.zeros(3)
shared.servo.lleg = vector.zeros(3)
shared.servo.rleg = vector.zeros(3)
shared.servo.imu = vector.zeros(17)

--Leg bias and walk params info
shared.leg={}
shared.leg.bias=vector.zeros(12)

-- Storing current stance info
shared.stance = {}
shared.stance.bodyTilt   = vector.zeros(1)
shared.stance.bodyHeight = vector.zeros(1)
shared.stance.bodyHeightTarget = vector.zeros(1)
shared.stance.uTorsoComp = vector.zeros(2) --For quasi-static balancing
shared.stance.uTorsoCompBias = vector.zeros(2)

shared.stance.waistPitchBias = vector.zeros(1) --To cancel out body sag
shared.stance.waistPitchBiasTarget = vector.zeros(1) --To cancel out body sag


--Used for drilling task
--Torso compensation is used to follow arm position, not for balancing
--0: compensation
--1: follow the arm
--2: stop compensation (keep current value)
shared.stance.enable_torso_track = vector.zeros(1)
shared.stance.track_hand_isleft = vector.zeros(1)
shared.stance.track_hand_y0 =  vector.zeros(1)
shared.stance.track_torso_y0 =  vector.zeros(1)

--Arm info

shared.arm = {}

--Target arm position (w/o compensation)
shared.arm.qlarm = vector.zeros(7)
shared.arm.qrarm = vector.zeros(7)

--Torso-Compensated arm position
shared.arm.qlarmcomp = vector.zeros(7)
shared.arm.qrarmcomp = vector.zeros(7)

--Current arm velocity limit
shared.arm.dqVelLeft = vector.zeros(6) --linear vel
shared.arm.dpVelLeft = vector.zeros(7) --joint vel

shared.arm.dqVelRight = vector.zeros(6)
shared.arm.dpVelRight = vector.zeros(7)

--hand offset X and Y (for hook)
shared.arm.handoffset = vector.new(Config.arm.handoffset.gripper)

shared.arm.lhandoffset = vector.new(Config.arm.handoffset.gripper)
shared.arm.rhandoffset = vector.new(Config.arm.handoffset.gripper)


-- Walk Parameters (for tuning on the fly)
shared.walk = {}
shared.walk.tStep      = vector.zeros(1)
shared.walk.tZmp 	   = vector.zeros(1)
shared.walk.bodyHeight = vector.zeros(1)
shared.walk.stepHeight = vector.zeros(1)
shared.walk.torsoX     = vector.zeros(1)
shared.walk.footY      = vector.zeros(1)
shared.walk.supportX   = vector.zeros(1)
shared.walk.supportY   = vector.zeros(1)
shared.walk.hipRollCompensation = vector.zeros(1)






--Walk state variables
shared.walk.bodyOffset = vector.zeros(3)
shared.walk.vel        = vector.zeros(3)
shared.walk.bipedal    = vector.zeros(1)
shared.walk.stoprequest= vector.zeros(1)
shared.walk.ismoving= vector.zeros(1)

-- Walk-step transition
shared.walk.steprequest= vector.zeros(1)
shared.walk.step_supportleg= vector.zeros(1)

-- kick type
shared.walk.kickphase= vector.zeros(1)
shared.walk.kicktype = vector.zeros(1)
shared.walk.kickfoot = vector.zeros(1)





-- Motion Status
shared.status = {}
shared.status.velocity   = vector.zeros(3)
shared.status.odometry   = vector.zeros(3)
shared.status.bodyOffset = vector.zeros(3)
shared.status.falling    = vector.zeros(1)

--Current Foot and Torso Poses
shared.status.uLeft = vector.zeros(3)
shared.status.uRight = vector.zeros(3)
shared.status.uTorso = vector.zeros(3)
shared.status.uSupport = vector.zeros(3)
shared.status.supportLeg = vector.zeros(1)
shared.status.ph = vector.zeros(1)


--We store the torso velocity (to handle stopping)
shared.status.uTorsoVel = vector.zeros(3)

--ZMP is stored here for external monitoring
shared.status.uZMP = vector.zeros(3)

--We monitor whole mass points here
shared.status.com_pos = vector.zeros(66)

--We keep 2 previous frames of joint angles 
--to calculate accelleration
shared.status.com_pos0 = vector.zeros(66)
shared.status.com_pos1 = vector.zeros(66)
shared.status.t0 = vector.zeros(1)
shared.status.t1 = vector.zeros(1)



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




local maxSteps = 8
shared.step = {}

--Footsteps queue
--{[rx ry ra supportLeg t0 t1 t2 zmpmodx zmpmody zmpmoda stepparam1 stepparam2 stepparam3]}
shared.step.footholds  = vector.zeros(13*maxSteps)
shared.step.nfootholds = vector.zeros(1)



memory.init_shm_segment(..., shared, shsize)
