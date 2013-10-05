local vector = require'vector'
local DEG_TO_RAD = math.pi/180

local Config = {}

------------------------------------
-- Walk Parameters
local walk = {}

------------------------------------
-- Stance and velocity limit values
------------------------------------
-- NOTE: Large stride test (up to 300mm)
walk.stanceLimitX = {-0.50,0.50}
walk.stanceLimitY = {0.16,0.60}
walk.stanceLimitA = {-10*math.pi/180,30*math.pi/180}
-- TODO: Toe/heel overlap checking values
--OP default stance width: 0.0375*2 = 0.075
--Heel overlap At radian 0.15 at each foot = 0.05*sin(0.15)*2=0.015
--Heel overlap At radian 0.30 at each foot = 0.05*sin(0.15)*2=0.030

walk.velLimitX = {-.15,.15}
walk.velLimitY = {-.08,.08}
walk.velLimitA = {-.3,.3}
walk.velDelta  = {0.05,0.03,0.3}
------------------------------------
-- Stance parameters
------------------------------------
walk.bodyHeight = 0.95
walk.bodyTilt = 0*math.pi/180
walk.supportX = 0.02  -- ankle-to-foot-center offset 
walk.supportY = 0.01  -- ankle-to-foot-center offset
walk.footY  = 0.095   -- body-center-to-ankle width
walk.torsoX = 0.00    -- com-to-body-center offset
------------------------------------
-- Gait parameters
------------------------------------
walk.tStep = 0.8
walk.tZMP = 0.33
walk.stepHeight = 0.04
walk.phSingle = {0.2,0.8}
walk.phZmp = {0.2,0.8}

------------------------------------
-- Compensation parameters
------------------------------------
walk.hardnessSupport = 1
walk.hardnessSwing = 1
--walk.hipRollCompensation = 3*math.pi/180
walk.hipRollCompensation = 1*math.pi/180
walk.supportModYInitial = -0.04 --Reduce initial body swing

-- slow/static
walk.supportY = 0.04
walk.bodyHeight = 0.950
walk.supportX = 0.01
walk.hipRollCompensation = 3*math.pi/180
walk.stepHeight = 0.02

walk.tZMP = 0.30
walk.footY  = 0.10
walk.supportY = 0.02
walk.supportX = 0.02
walk.stepHeight = 0.025

walk.hipRollCompensation = 1*math.pi/180

--Param for actual robot
gyroFactorX = 0.2
gyroFactorY = 0.2
walk.ankleImuParamX={0.3, 0.75*gyroFactorX,  1*math.pi/180, 5*math.pi/180}
walk.kneeImuParamX={0.3,1.5*gyroFactorX, 1*math.pi/180, 5*math.pi/180}
walk.ankleImuParamY = {0.3, 0.75*gyroFactorY, 2*math.pi/180, 5*math.pi/180}
walk.hipImuParamY   = { 0.3, 0.5*gyroFactorY, 2*math.pi/180, 5*math.pi/180}

walk.foot_traj = 1;


if IS_WEBOTS then
  --Webots parameters

  walk.tStep = 0.8
  walk.phSingle = {0.2,0.8}
  walk.phZmp = {0.2,0.8}
  walk.stepHeight = 0.05


--Robotis style walk
  walk.tStep = 0.45
  walk.phSingle = {0.125,0.875}
  walk.phZmp = {0.125,0.875}
  walk.stepHeight = 0.03

  gyroFactorX = 0.1
  gyroFactorY = 0.1
  walk.ankleImuParamX={0.3, 0.9*gyroFactorX, 1*math.pi/180, 5*math.pi/180}
  walk.kneeImuParamX={0.3,  0.3*gyroFactorX,1*math.pi/180, 5*math.pi/180}
  walk.ankleImuParamY = {0.3,1.0*gyroFactorY, 1*math.pi/180, 5*math.pi/180}
  walk.hipImuParamY = {0.3,  0.5*gyroFactorY, 1*math.pi/180, 5*math.pi/180}

  walk.foot_traj = 2; --square step





  --Robotis style walk
  walk.bodyHeight = 0.9285318
  walk.supportX = 0.0515184
  walk.footY = 0.1095
  walk.bodyTilt = 11*math.pi/180

  gyroFactorX = 490.23/(251000/180)*0
  gyroFactorY = 490.23/(251000/180)*0
  walk.ankleImuParamX={1, 0.9*gyroFactorX,  0*math.pi/180, 5*math.pi/180}
  walk.kneeImuParamX= {1, 0.3*gyroFactorX,    0*math.pi/180, 5*math.pi/180}
  walk.ankleImuParamY={1, 1.0*gyroFactorY,  0*math.pi/180, 5*math.pi/180}
  walk.hipImuParamY  ={1, 0.5*gyroFactorY,  0*math.pi/180, 5*math.pi/180}

 ------------------------------------

--Robotis style walk
  walk.tStep = 0.45
  walk.phSingle = {0.15,0.85}
  walk.phZmp = {0.15,0.85}
  walk.stepHeight = 0.04

  walk.supportX = 0.0515
  walk.supportY = 0.03
  walk.tZMP = 0.28

  walk.velLimitX = {-.20,.20}
  walk.velLimitY = {-.15,.15}

 --[[
 walk.phZmp = {0.10,0.90}
 walk.phSingle = {0.15,0.85}
 walk.tStep = 30
 walk.hipRollCompensation = 10*math.pi/180
 --]]
 





else

  --[[
    --FOR STATIC WALKING
    walk.hipRollCompensation = 2*math.pi/180
    walk.stepHeight = 0.03
    walk.bodyHeight = 0.93
    walk.supportY = 0.035 
    walk.footY  = 0.095
    walk.tStep = 12
  --]]

--Robotis default walk parameters

  walk.bodyHeight = 0.9285318
  walk.supportX = 0.0515184
  walk.footY = 0.1095
  walk.bodyTilt = 11*math.pi/180

  gyroFactorX = 490.23/(251000/180)*0.2
  gyroFactorY = 490.23/(251000/180)*0.2

  gyroFactorX = 490.23/(251000/180)*0.5


  walk.ankleImuParamX={1, 0.9*gyroFactorX,  0*math.pi/180, 5*math.pi/180}
  walk.kneeImuParamX= {1, -0.3*gyroFactorX,  0*math.pi/180, 5*math.pi/180}
  walk.ankleImuParamY={1, 1.0*gyroFactorY,  0*math.pi/180, 5*math.pi/180}
  walk.hipImuParamY  ={1, 0.5*gyroFactorY,  0*math.pi/180, 5*math.pi/180}




  walk.ankleImuParamX={1, 0.9*gyroFactorX,  1*math.pi/180, 5*math.pi/180}
  walk.kneeImuParamX= {1, -0.3*gyroFactorX,  1*math.pi/180, 5*math.pi/180}





  walk.ankleImuParamY={1, 1.0*gyroFactorY,  1*math.pi/180, 5*math.pi/180}
  walk.hipImuParamY  ={1, 0.5*gyroFactorY,  1*math.pi/180, 5*math.pi/180}

  gyroFactorY = 490.23/(251000/180)*0.5
--testing
  walk.ankleImuParamY={1, 1.0*gyroFactorY,  1*math.pi/180, 5*math.pi/180}
  walk.hipImuParamY  ={1, 0.5*gyroFactorY,  2*math.pi/180, 5*math.pi/180}


 ------------------------------------

--Robotis style walk
  walk.tStep = 0.45
  walk.phSingle = {0.125,0.875}
  walk.phZmp = {0.125,0.875}
  walk.stepHeight = 0.04

  walk.supportX = 0.01

  walk.tZMP = 0.30

--------------------------------------------
-- Humblewalk testing
-- This works kinda well with robotis feedback
  walk.tZMP = 0.30  --0.90m COM height
  walk.supportX = 0.03
  walk.supportY = 0.03
  walk.hipRollCompensation = 1*math.pi/180
---------------------------------------------

  walk.tZMP = 0.28 --0.80m COM height
  walk.supportY = 0.00
  walk.supportX = 0.04

--[[
--------------------------------------------
--Slow walk test
--Kinda stable with rollCompensation 
  walk.tZMP = 0.26 
  walk.tStep = 0.70
  walk.supportY = 0.04
  walk.phSingle = {0.2,0.8}
  walk.hipRollCompensation = 3*math.pi/180
--------------------------------------------
--]]


--[[
-------------------------------------------
--Even slower walk test
--Kinda stable too
walk.tStep = 1.0 

--------------------------------------------
--]]
end



-----------------------------------------------------------
-- Stance parameters
-----------------------------------------------------------


local stance={}
stance.enable_sit = false
stance.enable_legs = true   -- centaur has no legs

stance.pLLeg = vector.new{-walk.supportX,  walk.footY, 0, 0,0,0}
stance.pRLeg = vector.new{-walk.supportX, -walk.footY, 0, 0,0,0}
stance.pTorso = vector.new{-walk.torsoX, 0, walk.bodyHeight, 0,walk.bodyTilt,0}
stance.qWaist = vector.zeros(2)

stance.dqWaistLimit = 10*DEG_TO_RAD*vector.ones(2)
--stance.dpLimitStance = vector.new{.04, .03, .07, .4, .4, .4}
stance.dpLimitStance = vector.new{.04, .03, .03, .4, .4, .4}
stance.dqLegLimit = vector.new{10,10,45,90,45,10}*DEG_TO_RAD
--stance.dqLegLimit = vector.new{10,10,20,40,20,10}*DEG_TO_RAD

stance.sitHeight = 0.70
stance.dHeight = 0.04 --4cm per sec


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
-- For the arm FSM
local arm = {}
arm.qLArmInit={
 vector.new({110,12,-3,-40,  -0,0,0})*DEG_TO_RAD, -- at sides
 vector.new({110.5, 17.5, 0, -85.7, -30.2,  0,16.8})*DEG_TO_RAD, -- scarecrow
 vector.new({110.5, 17.5, -24, -85.7, -30.2, -71.0,16.8})*DEG_TO_RAD,-- arms in front
}
arm.qRArmInit={
 vector.new({110,-12,3,-40,     0,0,0})*DEG_TO_RAD, -- at sides
 vector.new({110.5, -17.5, 0, -85.7,  30.2,  0,-16.8})*DEG_TO_RAD,  -- scarecrow
 vector.new({110.5, -17.5, 24, -85.7, 30.2, 71.0,-16.8})*DEG_TO_RAD,-- arms in front
}


--[[

--Now ROCKY pose!
arm.qLArmInit={
 vector.new({90,0,-0,-160,  -90,-20,90})*DEG_TO_RAD, -- at sides
 vector.new({110.5, 17.5, 0, -85.7, -30.2,  0,16.8})*DEG_TO_RAD, -- scarecrow
 vector.new({110.5, 17.5, -24, -85.7, -30.2, -71.0,16.8})*DEG_TO_RAD,-- arms in front
}
arm.qRArmInit={
 vector.new({90,-0,0,-160,     90,20,-90})*DEG_TO_RAD, -- at sides
 vector.new({110.5, -17.5, 0, -85.7,  30.2,  0,-16.8})*DEG_TO_RAD,  -- scarecrow
 vector.new({110.5, -17.5, 24, -85.7, 30.2, 71.0,-16.8})*DEG_TO_RAD,-- arms in front
}

--]]




-- Arm speed limits
arm.fast_limit = vector.new({30,30,30,45,60,60,60})*DEG_TO_RAD
arm.slow_limit = vector.new({10,10,10,15,30,30,30})*DEG_TO_RAD
arm.super_slow_limit = vector.new({5,5,5,10,15,15,15})*DEG_TO_RAD
arm.slow_elbow_limit = vector.new({10,10,10,5,30,30,30})*DEG_TO_RAD

-- Linear movement speed limits
arm.linear_slow_limit = vector.new({0.02,0.02,0.02,
						15*DEG_TO_RAD,15*DEG_TO_RAD,15*DEG_TO_RAD})


--
if IS_WEBOTS then

--HACK FOR WEBOTS--------------------------------------------------
arm.fast_limit = vector.new({30,30,30,45,60,60,60})*DEG_TO_RAD*3
arm.slow_limit = vector.new({10,10,10,15,30,30,30})*DEG_TO_RAD*3
arm.super_slow_limit = vector.new({5,5,5,10,15,15,15})*DEG_TO_RAD*3
arm.slow_elbow_limit = vector.new({10,10,10,5,30,30,30})*DEG_TO_RAD*3

arm.linear_slow_limit = vector.new({0.02,0.02,0.02,
						15*DEG_TO_RAD,15*DEG_TO_RAD,15*DEG_TO_RAD})*3


end
--

arm.linear_wrist_limit = 0.05


------------------------------------
-- Associate with the table
Config.kneel   = kneel
Config.walk    = walk
Config.arm     = arm
Config.stance  = stance
Config.zmpstep = zmpstep

return Config
