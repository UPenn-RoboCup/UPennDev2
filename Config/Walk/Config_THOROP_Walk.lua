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

  --walk.supportX = 0.0515
  walk.supportX = 0.01
  walk.supportY = 0.03
  walk.tZMP = 0.28

  walk.velLimitX = {-.20,.20}
  walk.velLimitY = {-.15,.15}



--Slow walk
walk.tStep = 0.80
walk.supportY = 0.04
walk.tZMP = 0.33
walk.phSingle = {0.15,0.85}
walk.phZmp = {0.15,0.85}
walk.hipRollCompensation = 3*math.pi/180

--For webots
walk.supportY = 0.03
walk.tZMP = 0.28
walk.hipRollCompensation = 1*math.pi/180 --less compensation for webots


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


--Slow walk
walk.tStep = 0.80
walk.supportY = 0.04
walk.tZMP = 0.33
walk.phSingle = {0.15,0.85}
walk.phZmp = {0.15,0.85}
walk.hipRollCompensation = 3*math.pi/180

--[[
--Extremely slow walk
walk.tStep = 2.0
walk.phSingle = {0.2,.8}
walk.phZmp = {0.2,0.8}
walk.hipRollCompensation = 3*math.pi/180
--]]

end




--Slowed down walk (to match actual robot)
walk.velLimitX = {-.05,.05}
walk.velLimitY = {-.025,.025}
walk.velLimitA = {-.2,.2}
walk.velDelta  = {0.025,0.02,0.1}



--SJ:block walking test
--walk.stepHeight = 0.20

local stance={}

--TEMPORARY HACK FOR PERCEPTION TESTING
walk.bodyTilt = 11*math.pi/180
--walk.bodyTilt = 0*math.pi/180

--Should we move torso back for compensation?
stance.enable_torso_compensation = 1
stance.enable_torso_compensation = 0


--Temporary testing
--walk.bodyHeight = 0.97 --5cm higher

-----------------------------------------------------------
-- Stance parameters
-----------------------------------------------------------



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
zmpstep.param_k1_px={-576.304158,-389.290169,-68.684425}
zmpstep.param_a={
  {1.000000,0.010000,0.000050},
  {0.000000,1.000000,0.010000},
  {0.000000,0.000000,1.000000},
}
zmpstep.param_b={0.000000,0.000050,0.010000,0.010000}
zmpstep.param_k1={
    362.063442,105.967958,16.143995,-14.954972,-25.325109,
    -28.390091,-28.892019,-28.505769,-27.822565,-27.050784,
    -26.263560,-25.486256,-24.727376,-23.989477,-23.273029,
    -22.577780,-21.903233,-21.248818,-20.613950,-19.998050,
    -19.400555,-18.820915,-18.258596,-17.713080,-17.183867,
    -16.670467,-16.172409,-15.689235,-15.220498,-14.765769,
    -14.324628,-13.896668,-13.481497,-13.078731,-12.688001,
    -12.308946,-11.941218,-11.584477,-11.238396,-10.902656,
    -10.576948,-10.260972,-9.954438,-9.657062,-9.368572,
    -9.088702,-8.817195,-8.553799,-8.298273,-8.050383,
    -7.809898,-7.576600,-7.350271,-7.130705,-6.917699,
    -6.711057,-6.510589,-6.316110,-6.127442,-5.944411,
    -5.766848,-5.594590,-5.427479,-5.265360,-5.108085,
    -4.955508,-4.807490,-4.663894,-4.524588,-4.389444,
    -4.258337,-4.131147,-4.007757,-3.888053,-3.771925,
    -3.659267,-3.549974,-3.443946,-3.341085,-3.241298,
    -3.144491,-3.050577,-2.959468,-2.871080,-2.785333,
    -2.702148,-2.621447,-2.543158,-2.467207,-2.393524,
    -2.322043,-2.252697,-2.185423,-2.120158,-2.056843,
    -1.995419,-1.935830,-1.878021,-1.821939,-1.767532,
    -1.714751,-1.663545,-1.613870,-1.565678,-1.518926,
    -1.473570,-1.429569,-1.386883,-1.345471,-1.305297,
    -1.266322,-1.228512,-1.191831,-1.156246,-1.121723,
    -1.088232,-1.055741,-1.024221,-0.993642,-0.963977,
    -0.935197,-0.907277,-0.880192,-0.853915,-0.828423,
    -0.803692,-0.779701,-0.756425,-0.733845,-0.711940,
    -0.690689,-0.670072,-0.650071,-0.630668,-0.611844,
    -0.593583,-0.575867,-0.558680,-0.542007,-0.525831,
    -0.510139,-0.494915,-0.480146,-0.465818,-0.451919,
    -0.438434,-0.425352,-0.412661,-0.400349,-0.388404,
    -0.376817,-0.365575,-0.354670,-0.344090,-0.333826,
    -0.323868,-0.314208,-0.304837,-0.295745,-0.286925,
    -0.278368,-0.270067,-0.262014,-0.254201,-0.246622,
    -0.239269,-0.232135,-0.225214,-0.218500,-0.211987,
    -0.205668,-0.199538,-0.193590,-0.187820,-0.182223,
    -0.176792,-0.171523,-0.166412,-0.161453,-0.156642,
    -0.151975,-0.147446,-0.143053,-0.138791,-0.134655,
    -0.130643,-0.126750,-0.122974,-0.119309,-0.115754,
    -0.112305,-0.108958,-0.105710,-0.102559,-0.099502,
    -0.096535,-0.093657,-0.090863,-0.088153,-0.085523,
    -0.082970,-0.080493,-0.078089,-0.075756,-0.073491,
    -0.071293,-0.069160,-0.067089,-0.065079,-0.063128,
    -0.061234,-0.059394,-0.057609,-0.055875,-0.054191,
    -0.052556,-0.050968,-0.049425,-0.047926,-0.046470,
    -0.045056,-0.043681,-0.042345,-0.041046,-0.039783,
    -0.038556,-0.037362,-0.036200,-0.035070,-0.033971,
    -0.032900,-0.031858,-0.030843,-0.029855,-0.028892,
    -0.027953,-0.027037,-0.026144,-0.025273,-0.024422,
    -0.023591,-0.022779,-0.021986,-0.021209,-0.020450,
    -0.019706,-0.018977,-0.018262,-0.017561,-0.016873,
    -0.016197,-0.015532,-0.014877,-0.014233,-0.013597,
    -0.012970,-0.012351,-0.011739,-0.011133,-0.010533,
    -0.009938,-0.009347,-0.008760,-0.008176,-0.007594,
    -0.007013,-0.006433,-0.005854,-0.005274,-0.004692,
    -0.004108,-0.003522,-0.002932,-0.002338,-0.001739,
    -0.001134,-0.000522,0.000097,0.000724,0.001360,
    0.002006,0.002663,0.003331,0.004012,0.004706,
    0.005414,0.006138,0.006877,0.007632,0.008400,
    0.009175,0.009932,0.010604,0.010998,0.010561,
    0.007734,-0.001906,-0.030876,-0.114606,-0.353382,
    }


------------------------------------
-- For the arm FSM
local arm = {}

--Default gripper
arm.handoffset={}
arm.handoffset.gripper = {0.245,0.035,0}

--Single hook (for door)
arm.handoffset.outerhook = {0.285,-0.065,0}

--Two rod (for valve)
arm.handoffset.chopstick = {0.285,0,0}








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



-- Arm speed limits
arm.fast_limit = vector.new({30,30,30,45,60,60,60})*DEG_TO_RAD
arm.slow_limit = vector.new({10,10,10,15,30,30,30})*DEG_TO_RAD
arm.super_slow_limit = vector.new({5,5,5,10,15,15,15})*DEG_TO_RAD
arm.slow_elbow_limit = vector.new({10,10,10,5,30,30,30})*DEG_TO_RAD

-- Linear movement speed limits
arm.linear_slow_limit = vector.new({0.02,0.02,0.02, 15*DEG_TO_RAD,15*DEG_TO_RAD,15*DEG_TO_RAD})

-- Use this for wrist initialization
arm.joint_init_limit=vector.new({30,30,30,30,30,30,30}) *DEG_TO_RAD

-- Use this for planned arm movement
arm.joint_vel_limit_plan = vector.new({10,10,10,10,30,10,30}) *DEG_TO_RAD






--Pose 1 wrist position
arm.pLWristTarget1 = {.04,.30,-.20,0,0,0}
arm.pRWristTarget1 = {.04,-.30,-.20,0,0,0}

--lil higher and back
--arm.pLWristTarget1 = {.00,.30,-.15,0,0,0}
--arm.pRWristTarget1 = {.00,-.30,-.15,0,0,0}

arm.lShoulderYawTarget1 = -5*DEG_TO_RAD
arm.rShoulderYawTarget1 = 5*DEG_TO_RAD

--Pose 2 wrist position
arm.pLWristTarget2 = {.05,.38,-.05,0,0,0}
arm.pRWristTarget2 = {.05,-.38,-.05,0,0,0}
arm.lShoulderYawTarget2 = -20*DEG_TO_RAD
arm.rShoulderYawTarget2 = 20*DEG_TO_RAD
arm.linear_wrist_limit = 0.05

if IS_WEBOTS then

    --Faster limit for webots
  arm.fast_limit = arm.fast_limit*3
  arm.slow_limit = arm.slow_limit*3
  arm.super_slow_limit = arm.super_slow_limit*3
  arm.slow_elbow_limit = arm.slow_elbow_limit*3

  -- Linear movement speed limits
  arm.linear_slow_limit = arm.linear_slow_limit*3

  -- Use this for wrist initialization
  arm.joint_init_limit=arm.joint_init_limit*3

  -- Use this for planned arm movement
  arm.joint_vel_limit_plan = arm.joint_vel_limit_plan*3





  --For low-torque deflection testing
--[[
  walk.ankleImuParamX={0, 0.9*gyroFactorX,  1*math.pi/180, 5*math.pi/180}
  walk.kneeImuParamX= {0, -0.3*gyroFactorX,  1*math.pi/180, 5*math.pi/180}
  walk.ankleImuParamY={0, 1.0*gyroFactorY,  1*math.pi/180, 5*math.pi/180}
  walk.hipImuParamY  ={0, 0.5*gyroFactorY,  2*math.pi/180, 5*math.pi/180}
--]]
  walk.hipRollCompensation = 1*math.pi/180
  walk.ankleRollCompensation = 1.2*math.pi/180
  walk.hipPitchCompensation = 0*math.pi/180
  walk.kneePitchCompensation = 0*math.pi/180
  walk.anklePitchCompensation = 0*math.pi/180
  walk.phComp = {0.1,0.9}
  walk.phCompSlope = 0.2
else


  --Compensation testing values
  walk.hipRollCompensation = 1*math.pi/180
  walk.ankleRollCompensation = 1.2*math.pi/180
  walk.hipPitchCompensation = -1.0*math.pi/180
  walk.kneePitchCompensation = 1.0*math.pi/180
  walk.anklePitchCompensation = 1.5*math.pi/180

  --No compensation testing
  --
  walk.hipRollCompensation = 3*math.pi/180
  walk.ankleRollCompensation = 0*math.pi/180
  walk.hipPitchCompensation = 0*math.pi/180
  walk.kneePitchCompensation = 0*math.pi/180
  walk.anklePitchCompensation = 0*math.pi/180
  --
  walk.phComp = {0.1,0.9}
  walk.phCompSlope = 0.2
end



walk.leg_p_gain = 64



--FOR testing (for conference)
--[[
walk.tStep = 0.80
walk.supportY = 0.03
walk.tZMP = 0.33

walk.velLimitX = {-.30,.30}
walk.velDelta  = {0.15,0.03,0.3}

--shut down feedback
walk.ankleImuParamX[1]=0
walk.kneeImuParamX[1]=0
walk.ankleImuParamY[1]=0
walk.hipImuParamY[1]=0

walk.accTorso = 1.5 --fast stop
--]]


------------------------------------
-- Associate with the table
Config.kneel   = kneel
Config.walk    = walk
Config.arm     = arm
Config.stance  = stance
Config.zmpstep = zmpstep

return Config
