local vector = require'vector'
local DEG_TO_RAD = math.pi/180

local Config = {}
------------------------------------
-- Kneeling parameters
------------------------------------
local kneel = {}
kneel.bodyHeight = 0.45
kneel.bodyTilt = 90*math.pi/180

kneel.ph1Single = 0.2
kneel.ph2Single = 0.8
kneel.tZMP = 0.22

kneel.tStep = 0.50
kneel.stepHeight = 0.15

kneel.velLimitX = {-.10,.20}
kneel.velLimitY = {-.5,.5}
kneel.velLimitA = {-.2,.2}

kneel.armX = 0.35
kneel.armY = 0.296
kneel.armZ = 0.078
kneel.LArmRPY = {-math.pi/2,0,0}
kneel.RArmRPY = {math.pi/2,0,0}
kneel.legX = -0.395
kneel.legY = 0.210
--only used for kneel down 
kneel.qLArm0 = {0.43,0.26,0.09,-1.15,-1.57,-1.57}

-- For wrist-walking 
kneel.armX = 0.26
kneel.armZ = 0.02
-- High step testing
kneel.stepHeight = 0.20
kneel.armX = 0.21

--[[
--Higher step testing
kneel.tStep = 1
kneel.ph1Single = 0.1
kneel.ph2Single = 0.9
--]]

kneel.torsoX = -(kneel.armX + kneel.legX)/2

------------------------------------
-- ZMP preview stepping values
------------------------------------
local zmpstep = {}
zmpstep.bodyHeight = 0.98 
zmpstep.bodyTilt = 0
zmpstep.tZMP = 0.28 
zmpstep.supportX = 0.02
zmpstep.supportY = 0.0
zmpstep.stepHeight = 0.10
zmpstep.phSingle={0.1,0.9}
zmpstep.hipRollCompensation = 3*math.pi/180
--zmpstep.bodyHeight = 1.10 

------------------------------------
-- Walk Parameters
local walk = {}

------------------------------------
-- Stance and velocity limit values
------------------------------------
-- NOTE: Large stride test (up to 300mm)
walk.stanceLimitX = {-0.60,0.60}
walk.stanceLimitY = {0.16,0.60}
walk.stanceLimitA = {-10*math.pi/180,30*math.pi/180}
-- TODO: Toe/heel overlap checking values
--OP default stance width: 0.0375*2 = 0.075
--Heel overlap At radian 0.15 at each foot = 0.05*sin(0.15)*2=0.015
--Heel overlap At radian 0.30 at each foot = 0.05*sin(0.15)*2=0.030

walk.velLimitX = {-.20,.30}
walk.velLimitY = {-.20,.20}
walk.velLimitA = {-.3,.3}
walk.velDelta  = {0.15,0.10,0.3}

------------------------------------
-- Stance parameters
------------------------------------
walk.bodyHeight = 1.15
walk.bodyTilt = 0*math.pi/180
-- footX is deprecated in favor of torsoX
walk.torsoX = 0.00
walk.footY = 0.10
walk.supportX = 0.03
walk.supportY = 0.02
walk.qLArm = math.pi/180*vector.new({110, 12, -0, -40,0,0})
walk.qRArm = math.pi/180*vector.new({110, -12, 0, -40,0,0})

walk.qLArmKick = math.pi/180*vector.new({110, 12, -0, -40,0,0})
walk.qRArmKick = math.pi/180*vector.new({110, -12, 0, -40,0,0})

walk.hardnessSupport = 1
walk.hardnessSwing = 1
walk.hardnessArm = .1
------------------------------------
-- Gait parameters
------------------------------------
walk.stepHeight = 0.052
walk.tZMP = 0.30 --Com height 0.9
walk.tStep = 0.8
walk.phSingle = {0.15,0.85}
walk.phZmp = {0.15,0.85}

------------------------------------
-- Compensation parameters
------------------------------------
walk.hipRollCompensation = 1*math.pi/180
walk.ankleMod = vector.new({-1,0})/ 3*math.pi/180
walk.supportModYInitial = -0.04 --Reduce initial body swing

-----------------------------------------------------------
--Imu feedback parameters, alpha / gain / deadband / max --
-----------------------------------------------------------
gyroFactor = 0.273*math.pi/180 * 300 / 1024 --dps to rad/s conversion
-- We won't use gyro feedback on webots
gyroFactorX = gyroFactor * 0
gyroFactorY = gyroFactor * 0
--
walk.ankleImuParamX={0.3,0.75*gyroFactorX, 0*math.pi/180, 5*math.pi/180}
walk.ankleImuParamY={0.3,0.25*gyroFactorY, 0*math.pi/180, 2*math.pi/180}
--
walk.kneeImuParamX={0.3,1.5*gyroFactorX, 0*math.pi/180, 5*math.pi/180}
--
walk.hipImuParamY={0.3,0.25*gyroFactorY, 0*math.pi/180, 2*math.pi/180}
--
walk.armImuParamX={1,10*gyroFactorX, 20*math.pi/180, 45*math.pi/180}
walk.armImuParamY={1,10*gyroFactorY, 20*math.pi/180, 45*math.pi/180}

------------------------------------
-- WalkKick parameters
------------------------------------
walk.walkKickDef={}

------------------------------------
-- Sit/stand stance parameters
-- NOTE: Never sit down!

local stance={}
stance.enable_sit = false

stance.hardnessLeg = 1
stance.bodyHeightSit = 0.75
stance.supportXSit = -0.00
stance.bodyHeightDive = 0.65

--bodyInitial bodyTilt, 0 for webots
stance.bodyTiltStance = 0*math.pi/180
stance.dpLimitStance = vector.new({.4, .3, .4, .05, .4, .1})*0.6
stance.dpLimitStance = vector.new({.04, .03, .07, .4, .4, .4})
stance.dpLimitSit = vector.new({.1,.01,.06,.1,.3,.1})*2
stance.delay = 80 

------------------------------------
-- For the arm FSM
local arm = {}
arm.qLArmInit={
vector.new({90,0,90,-0,-90,0})*DEG_TO_RAD,
vector.new({90,90,90,-90,-90,0})*DEG_TO_RAD,
vector.new({0,90,90,-90,-90,-45})*DEG_TO_RAD,
}

arm.qRArmInit={
vector.new({90,-0,-90,-0,90,0})*DEG_TO_RAD, -- at sides
vector.new({90,-90,-90,-90,90,0})*DEG_TO_RAD,  -- scarecrow
vector.new({0,-90,-90,-90,90,45})*DEG_TO_RAD, -- arms in front
}

arm.FingerClosed = 0.9
arm.FingerOpen = 0.1

------------------------------------
-- Associate with the table
Config.kneel   = kneel
Config.walk    = walk
Config.arm     = arm
Config.stance  = stance
Config.zmpstep = zmpstep

return Config