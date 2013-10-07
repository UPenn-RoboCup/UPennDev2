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

  --To get rid of drifting
  walk.velocityBias = {0.005,0,0}


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


--SJ:block walking test
--walk.stepHeight = 0.20

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
zmpstep.param_r_q = 10^-6
zmpstep.preview_tStep = 0.010 --10ms
zmpstep.preview_interval = 3.0 --3s needed for slow step (with tStep ~3s)




zmpstep.params = true;
zmpstep.param_k1_px={-658.671153,-380.924007,-58.508050}
zmpstep.param_a={
  {1.000000,0.010000,0.000050},
  {0.000000,1.000000,0.010000},
  {0.000000,0.000000,1.000000},
}
zmpstep.param_b={0.000000,0.000050,0.010000,0.010000}
zmpstep.param_k1={
    340.070737,134.153746,39.047358,-4.448725,-23.922221,
    -32.228652,-35.358018,-36.100850,-35.756685,-34.930251,
    -33.902510,-32.803574,-31.693279,-30.598743,-29.531862,
    -28.497465,-27.497113,-26.530866,-25.598111,-24.697942,
    -23.829341,-22.991254,-22.182635,-21.402459,-20.649732,
    -19.923489,-19.222799,-18.546764,-17.894516,-17.265217,
    -16.658060,-16.072265,-15.507080,-14.961780,-14.435666,
    -13.928060,-13.438313,-12.965795,-12.509900,-12.070043,
    -11.645660,-11.236205,-10.841154,-10.459999,-10.092252,
    -9.737440,-9.395109,-9.064819,-8.746146,-8.438681,
    -8.142031,-7.855815,-7.579664,-7.313226,-7.056159,
    -6.808132,-6.568828,-6.337940,-6.115171,-5.900236,
    -5.692859,-5.492775,-5.299727,-5.113467,-4.933756,
    -4.760364,-4.593069,-4.431657,-4.275920,-4.125658,
    -3.980679,-3.840798,-3.705835,-3.575616,-3.449976,
    -3.328753,-3.211791,-3.098941,-2.990058,-2.885003,
    -2.783640,-2.685841,-2.591480,-2.500435,-2.412591,
    -2.327834,-2.246057,-2.167154,-2.091024,-2.017570,
    -1.946697,-1.878316,-1.812337,-1.748678,-1.687255,
    -1.627991,-1.570810,-1.515639,-1.462406,-1.411044,
    -1.361486,-1.313670,-1.267535,-1.223020,-1.180070,
    -1.138628,-1.098643,-1.060063,-1.022838,-0.986921,
    -0.952266,-0.918829,-0.886566,-0.855437,-0.825401,
    -0.796421,-0.768458,-0.741478,-0.715446,-0.690328,
    -0.666093,-0.642708,-0.620146,-0.598375,-0.577370,
    -0.557102,-0.537546,-0.518677,-0.500471,-0.482905,
    -0.465955,-0.449600,-0.433820,-0.418594,-0.403903,
    -0.389728,-0.376051,-0.362853,-0.350120,-0.337833,
    -0.325978,-0.314540,-0.303503,-0.292853,-0.282578,
    -0.272663,-0.263097,-0.253866,-0.244959,-0.236366,
    -0.228074,-0.220073,-0.212353,-0.204904,-0.197717,
    -0.190782,-0.184090,-0.177633,-0.171403,-0.165392,
    -0.159592,-0.153996,-0.148595,-0.143385,-0.138357,
    -0.133506,-0.128826,-0.124309,-0.119951,-0.115746,
    -0.111689,-0.107774,-0.103997,-0.100352,-0.096835,
    -0.093441,-0.090167,-0.087007,-0.083959,-0.081017,
    -0.078179,-0.075440,-0.072797,-0.070248,-0.067787,
    -0.065413,-0.063122,-0.060912,-0.058779,-0.056721,
    -0.054735,-0.052819,-0.050970,-0.049186,-0.047464,
    -0.045803,-0.044200,-0.042653,-0.041160,-0.039720,
    -0.038330,-0.036989,-0.035694,-0.034445,-0.033240,
    -0.032077,-0.030954,-0.029871,-0.028825,-0.027816,
    -0.026843,-0.025903,-0.024995,-0.024120,-0.023275,
    -0.022459,-0.021671,-0.020911,-0.020177,-0.019469,
    -0.018784,-0.018124,-0.017486,-0.016870,-0.016275,
    -0.015700,-0.015145,-0.014609,-0.014091,-0.013590,
    -0.013106,-0.012638,-0.012186,-0.011749,-0.011326,
    -0.010917,-0.010521,-0.010138,-0.009767,-0.009408,
    -0.009060,-0.008723,-0.008396,-0.008080,-0.007772,
    -0.007474,-0.007184,-0.006902,-0.006629,-0.006362,
    -0.006103,-0.005850,-0.005604,-0.005364,-0.005129,
    -0.004900,-0.004675,-0.004455,-0.004240,-0.004028,
    -0.003820,-0.003616,-0.003414,-0.003215,-0.003018,
    -0.002824,-0.002631,-0.002440,-0.002250,-0.002060,
    -0.001871,-0.001683,-0.001494,-0.001304,-0.001114,
    -0.000923,-0.000730,-0.000536,-0.000339,-0.000140,
    0.000062,0.000267,0.000476,0.000689,0.000906,
    0.001127,0.001352,0.001581,0.001811,0.002038,
    0.002250,0.002423,0.002503,0.002378,0.001801,
    0.000246,-0.003418,-0.011621,-0.029573,-0.068458,
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
