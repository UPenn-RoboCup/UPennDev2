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
--walk.hipRollCompensation = 3*math.pi/180
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
--[[
  walk.tStep = 0.8
  walk.phSingle = {0.2,0.8}
  walk.phZmp = {0.2,0.8}
  walk.stepHeight = 0.05
--]]

--Robotis style walk
  walk.tStep = 0.45
  walk.phSingle = {0.125,0.875}
  walk.phZmp = {0.125,0.875}
  walk.stepHeight = 0.04

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


--ZMP preview walk test

else

  walk.foot_traj = 1; --curved step
  walk.foot_traj = 2; --square step

--Robotis default walk parameters
  walk.bodyHeight = 0.9285318
  walk.supportX = 0.0515184
  walk.footY = 0.1095
  walk.bodyTilt = 11*math.pi/180

  gyroFactorX = 490.23/(251000/180)*0.5
  gyroFactorY = 490.23/(251000/180)*0.5
  walk.ankleImuParamX={1, 0.9*gyroFactorX,  1*math.pi/180, 5*math.pi/180}
  walk.kneeImuParamX= {1, -0.3*gyroFactorX,  1*math.pi/180, 5*math.pi/180}
  walk.ankleImuParamY={1, 1.0*gyroFactorY,  1*math.pi/180, 5*math.pi/180}
  walk.hipImuParamY  ={1, 0.5*gyroFactorY,  2*math.pi/180, 5*math.pi/180}
 ------------------------------------

--Robotis style walk w/ humblewalk
  walk.tStep = 0.45
  walk.phSingle = {0.125,0.875}
  walk.phZmp = {0.125,0.875}
  walk.stepHeight = 0.04

--------------------------------------------
-- This works kinda well
  walk.tZMP = 0.30  --0.90m COM height
  walk.supportX = 0.03
  walk.supportY = 0.03
  walk.hipRollCompensation = 1*math.pi/180
--------------------------------------------

--------------------------------------------
--Also works kinda well
  walk.tZMP = 0.28 --0.80m COM height
  walk.supportX = 0.04
  walk.supportY = 0.00
  ------------------------------------------

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

--SJ: now we use walk parmas to generate these
zmpstep.params = true;

zmpstep.param_r_q = 10^-6
zmpstep.preview_tStep = 0.010 --10ms
zmpstep.preview_interval = 1.50

zmpstep.param_k1_px={-435.038163,-315.470294,-57.649816}
zmpstep.param_a={
  {1.000000,0.010000,0.000050},
  {0.000000,1.000000,0.010000},
  {0.000000,0.000000,1.000000},
}
zmpstep.param_b={0.000000,0.000050,0.010000,0.010000}
zmpstep.param_k1={
    344.261835,140.150393,45.743023,2.435294,-17.082997,
    -25.538215,-28.860333,-29.812066,-29.680142,-29.063466,
    -28.240430,-27.340083,-26.421901,-25.512960,-24.625259,
    -23.763796,-22.930323,-22.125102,-21.347720,-20.597466,
    -19.873512,-19.174990,-18.501032,-17.850787,-17.223424,
    -16.618141,-16.034161,-15.470735,-14.927139,-14.402674,
    -13.896668,-13.408469,-12.937450,-12.483006,-12.044554,
    -11.621529,-11.213388,-10.819607,-10.439679,-10.073115,
    -9.719446,-9.378215,-9.048985,-8.731332,-8.424847,
    -8.129137,-7.843820,-7.568531,-7.302914,-7.046628,
    -6.799343,-6.560741,-6.330514,-6.108366,-5.894011,
    -5.687172,-5.487583,-5.294986,-5.109134,-4.929786,
    -4.756710,-4.589684,-4.428490,-4.272922,-4.122778,
    -3.977863,-3.837989,-3.702976,-3.572648,-3.446837,
    -3.325378,-3.208113,-3.094891,-2.985563,-2.879987,
    -2.778025,-2.679543,-2.584413,-2.492510,-2.403712,
    -2.317904,-2.234971,-2.154805,-2.077299,-2.002351,
    -1.929859,-1.859729,-1.791865,-1.726178,-1.662578,
    -1.600979,-1.541298,-1.483455,-1.427369,-1.372964,
    -1.320165,-1.268899,-1.219094,-1.170681,-1.123592,
    -1.077760,-1.033119,-0.989606,-0.947159,-0.905714,
    -0.865213,-0.825594,-0.786800,-0.748772,-0.711452,
    -0.674784,-0.638712,-0.603179,-0.568131,-0.533511,
    -0.499264,-0.465336,-0.431672,-0.398215,-0.364912,
    -0.331706,-0.298541,-0.265362,-0.232110,-0.198730,
    -0.165161,-0.131347,-0.097225,-0.062737,-0.027820,
    0.007587,0.043548,0.080124,0.117373,0.155342,
    0.194058,0.233500,0.273535,0.313802,0.353435,
    0.390493,0.420712,0.434857,0.413011,0.312334,
    0.040803,-0.599158,-2.031617,-5.166658,-11.957266,
    }

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
