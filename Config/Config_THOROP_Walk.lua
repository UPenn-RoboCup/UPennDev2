local vector = require'vector'
local DEG_TO_RAD = math.pi/180

local Config = {}

------------------------------------
-- Walk Parameters

local walk = {}

walk.init_command_velocity = 500
walk.init_command_accelleration = 50



walk.leg_p_gain = 64
walk.ankle_p_gain = 64

--Default-y vaue
walk.maxTurnSpeed = 0.10
walk.aTurnSpeed = 0.25
walk.maxStepCount = 8



print("Robot hostname:",HOSTNAME)

------------------------------------
-- Stance and velocity limit values
------------------------------------
walk.stanceLimitX = {-0.50,0.50}
--walk.stanceLimitY = {0.16,0.60}
walk.stanceLimitY = {0.16,0.30}
walk.stanceLimitA = {-10*math.pi/180,30*math.pi/180}

if IS_WEBOTS then  

  walk.maxTurnSpeed = 0.20
  walk.aTurnSpeed = 0.25
  walk.maxStepCount = 30



  walk.foot_traj = 2; --square step

  --Robotis style walk
  walk.bodyHeight = 0.9285318 
  walk.bodyTilt = 11*math.pi/180
  walk.footY = 0.1095
  walk.torsoX = 0.00    -- com-to-body-center offset

  walk.stepHeight = 0.04
  walk.supportX = 0.01
  walk.supportY = 0.03
  walk.tZMP = 0.28

  walk.tStep = 0.80
  walk.supportY = 0.03
  walk.tZMP = 0.28
  walk.phSingle = {0.15,0.85}
  walk.phZmp = {0.15,0.85}

  gyroFactorX = 490.23/(251000/180)*0
  gyroFactorY = 490.23/(251000/180)*0
  walk.ankleImuParamX={1, 0.9*gyroFactorX,  0*math.pi/180, 5*math.pi/180}
  walk.kneeImuParamX= {1, 0.3*gyroFactorX,    0*math.pi/180, 5*math.pi/180}
  walk.ankleImuParamY={1, 1.0*gyroFactorY,  0*math.pi/180, 5*math.pi/180}
  walk.hipImuParamY  ={1, 0.5*gyroFactorY,  0*math.pi/180, 5*math.pi/180}

  walk.hipRollCompensation = 1*math.pi/180
  walk.ankleRollCompensation = 1.2*math.pi/180
  walk.hipPitchCompensation = 0*math.pi/180
  walk.kneePitchCompensation = 0*math.pi/180
  walk.anklePitchCompensation = 0*math.pi/180
  walk.phComp = {0.1,0.9}
  walk.phCompSlope = 0.2
 
  
  walk.velLimitX = {-.05,.05}
  walk.velLimitY = {-.025,.025}
  walk.velLimitA = {-.2,.2}
  walk.velDelta  = {0.025,0.02,0.1}
else
------------------------------------
-- Stance parameters
------------------------------------

--Robotis default walk parameters
  walk.bodyHeight = 0.9285318
--  walk.supportX = 0.0515184
  walk.footY = 0.1095
  walk.bodyTilt = 11*math.pi/180
  walk.torsoX = 0.00    -- com-to-body-center offset

------------------------------------
-- Gait parameters
------------------------------------
  walk.tStep = 0.80
  walk.tZMP = 0.33
  walk.stepHeight = 0.04  
  walk.phSingle = {0.15,0.85}
  walk.phZmp = {0.15,0.85}
  walk.supportX = 0.03
  walk.supportY = 0.04  
------------------------------------
-- Compensation parameters
------------------------------------
  gyroFactorX = 490.23/(251000/180)*0.5
  gyroFactorY = 490.23/(251000/180)*0.5
  walk.ankleImuParamX={1, 0.9*gyroFactorX,  1*math.pi/180, 5*math.pi/180}
  walk.kneeImuParamX= {1, -0.3*gyroFactorX,  1*math.pi/180, 5*math.pi/180}
  walk.ankleImuParamY={1, 1.0*gyroFactorY,  1*math.pi/180, 5*math.pi/180}
  walk.hipImuParamY  ={1, 0.5*gyroFactorY,  2*math.pi/180, 5*math.pi/180}

  --Compensation testing values
  --[[
  walk.hipRollCompensation = 1*math.pi/180
  walk.ankleRollCompensation = 1.2*math.pi/180
  walk.hipPitchCompensation = -1.0*math.pi/180
  walk.kneePitchCompensation = 1.0*math.pi/180
  walk.anklePitchCompensation = 1.5*math.pi/180
  --]]

  walk.hipRollCompensation = 3*math.pi/180
  walk.ankleRollCompensation = 0*math.pi/180
  walk.hipPitchCompensation = 0*math.pi/180
  walk.kneePitchCompensation = 0*math.pi/180
  walk.anklePitchCompensation = 0*math.pi/180
  --
  walk.phComp = {0.1,0.9}
  walk.phCompSlope = 0.2










 ------------------------------------

  walk.foot_traj = 1; --curved step
  walk.foot_traj = 2; --square step

  --To get rid of drifting
  walk.velocityBias = {0.005,0,0}  
  walk.velocityBias = {0.00,0,0}  
  walk.velLimitX = {-.05,.05}
  walk.velLimitY = {-.02,.02}
  walk.velLimitA = {-.2,.2}
  walk.velDelta  = {0.025,0.02,0.1}
end


if HOSTNAME=="alvin" then


  walk.legBias = 
    vector.new({
      0,  1.0,  -1.0,  -1.0,  0, 0, --LLEG
      0,  -0.50, 0,0,0, 0,  --RLEG
    })*DEG_TO_RAD

--New bias 12/14 9:12 PM

  walk.legBias = 
    vector.new({
      0.875, 0.875, -0.625, -1.75, 0.00, -0.75,
      0.625, -0.50, 0.375, 0.125, 0.00, 0.125 
    })*DEG_TO_RAD


--For alvin
  walk.hipRollCompensation = 1*math.pi/180
  walk.stepHeight = 0.03
  walk.supportX = 0.02
  walk.supportY = 0.03

  walk.velLimitY = {-.01,.01}
  walk.velLimitA = {-.2,.2}
  walk.velDelta  = {0.025,0.02,0.1}

--with battery
  walk.torsoX = -0.02


--Narrower stance, robotis value, 12/14 10:10PM
  walk.footY = 0.095 
  walk.torsoX = -0.04 
  walk.hipRollCompensation = 2*math.pi/180
  walk.hipRollCompensationLeft = 2*math.pi/180
  walk.hipRollCompensationRight = 2*math.pi/180

  walk.supportY = 0.05


  walk.velLimitA = {-.4,.4}

  --JK's bias 10:54PM
--hiproll: 1 -0.4325
--kneepitch: -2.155 -0.3475
--ankleroll: -0.345 1.0025

  walk.legBias = 
    vector.new({
      0.875, 1, -0.625, -2.155, 0.00, -0.345,
      0.625, -0.4325, 0.375, -0.3475, 0.00, 1.0025 
    })*DEG_TO_RAD

  --Faster turn testing
  walk.maxTurnSpeed = 0.20

  walk.maxTurnSpeed = 0.10

  walk.aTurnSpeed = 0.25
  walk.maxStepCount = 30


  --ankle roll tweak
  walk.legBias = 
    vector.new({
      0.875, 1, -0.625, -2.155, 0.00, -0.615,
      0.625, -0.4325, 0.375, -0.3475, 0.00, 1.0025 
    })*DEG_TO_RAD

  walk.torsoX3 = -0.04 
  walk.torsoX = -0.02 



--19th, 9:02AM
  walk.legBias = 
    vector.new({
      0.875, 2.35, -0.625, -2.155,       0.00, -0.615,
      0.1525, -1.2425, 0.375, -0.3475,   0.00, 0.41
    })*DEG_TO_RAD


--D-day, 7:31AM

  walk.legBias = 
    vector.new({
      0.875,  2.1475, -0.625, -1.5475,       0.00, -0.615,
      -0.2525, -1.58, 1.1175, -0.3475,   0.00, 0.41
    })*DEG_TO_RAD




--hiproll: 2.1475 -1.58
--hippitch: -0.625 1.1175
--kneepitch: -1.5475 -0.3475


--hipyaw:0.875 -0.2525

  walk.legBias = 
    vector.new({
      0.875,  2.1475, -0.625, -1.5475,       1.2775, -0.615,
      -0.2525, -1.58, 1.1175, -0.3475,   0.940, 0.41
    })*DEG_TO_RAD



--AFTER HIP SWAP
--hiproll: 2.1475 -1.1075
--hippitch -0.8275 0.915
--kneepitch -1.9525 -0.7525

  walk.legBias = 
    vector.new({
      0.875,  2.1475, -0.8275, -1.9525,       1.2775, -0.615,
      -0.2525, -1.1075, 0.915, -0.752,   0.940, 0.41
    })*DEG_TO_RAD

--angle fune tuning

  walk.legBias = 
    vector.new({
      0.875,  2.1475, -0.49, -1.9525,       1.48, -0.615,
      -0.2525, -1.1075, 1.25, -0.752,   0.740, 0.41
    })*DEG_TO_RAD




--at penn, with broken hip servo
  walk.legBias = 
    vector.new({
      0.875,  2.1475, -0.49, -1.9525,       1.48, -1.22,
      -0.2525, -1.1075, 1.25, -0.752,   0.740, 1.15
    })*DEG_TO_RAD


elseif HOSTNAME=="teddy" then
--for teddy
  walk.hipRollCompensation = 1*math.pi/180
  walk.stepHeight = 0.03
  walk.supportX = 0.02
  walk.supportY = 0.03

  print("TEDDY")
  walk.legBias = 
    vector.new({1, 0.50,    -0, -0.25,  0.25, 0.25,
                0, 0,    0.75,   -1,  0, -0.50,
    })*DEG_TO_RAD
--JK bias 12/14

  --hiproll 0.25 0.125
  --hippitch -0.25 0.375
  --kneepitch -0.5 -0.875

  walk.legBias = 
    vector.new({1, 0.25,-0.25, -0.50,  0.25, 0.25,
                0, 0.125,0.375,-0.875,  0, -0.50,
    })*DEG_TO_RAD

--New values
  walk.footY = 0.095 
  walk.torsoX = -0.04 
  walk.hipRollCompensation = 2*math.pi/180

  walk.hipRollCompensationLeft = 2*math.pi/180
  walk.hipRollCompensationRight = 2*math.pi/180

  walk.supportY = 0.05

  walk.maxTurnSpeed = 0.20
  walk.aTurnSpeed = 0.25
  walk.maxStepCount = 30



--Little less torso shift (with hands on)
  walk.torsoX = -0.04 
  walk.torsoX = -0.02 

--ankle roll tweak
  walk.legBias = 
    vector.new({1, 0.25,-0.25, -0.50,  0.25, 0.5875,
                0, 0.125,0.375,-0.875,  0, -0.7025,
    })*DEG_TO_RAD

--ankle pitch tweak
  walk.legBias = 
    vector.new({1, 0.25,-0.25, -0.50,  0.52, 0.25,
                0, 0.125,0.375,-0.875,  0, -0.50,
    })*DEG_TO_RAD

--Support Y reduce
  walk.supportY = 0.05
  walk.supportY = 0.03

--ankle roll untweak

  walk.legBias = 
    vector.new({1, 0.25,-0.25, -0.50,  0.52, 0.25,
                0, 0.125,0.375,-0.875,  0, -0.50,
    })*DEG_TO_RAD


--Roll fix after collapse 

  walk.legBias = 
    vector.new({1, 1.26,-0.25, -0.50,  1.06, 0.2475,
                0, -0.415,0.375,-0.875,  0, -0.50,
    })*DEG_TO_RAD










--DRC site, 12/18 3:21PM
--HY  1 -0.27
--HP -.495 1.455
--AP -0.4925 -0.945
--AR .01125 0.175

  walk.legBias = 
    vector.new({1, 1.26,    0.495,   -0.50,  -0.4925, 0.1125,
            -0.27, -0.415,  1.455,   -0.875,  -0.945, 0.175,
    })*DEG_TO_RAD


  walk.torsoX = -0.02 
  walk.torsoX = -0.00 



  walk.supportY = 0.05
--Ankle tweak, 5:26PM
  walk.legBias = 
    vector.new({1, 1.26,    0.495,   -0.50,  -0.02, 0.5175,
            -0.27, -0.415,  1.455,   -0.875,  -0.945, 0.175,
    })*DEG_TO_RAD



  walk.maxTurnSpeed = 0.10

  --PENN DEMO, 1/22/2014
  walk.legBias = 
    vector.new({1, 1.26,    0.36,   -0.50,  -0.02, 0.5175,
            -0.27, -0.415,  1.05,   -0.27,  -0.945, 0.175,
    })*DEG_TO_RAD

-- BIAS set 1
  walk.legBias = 
    vector.new({1, 1.26,    0.36,   -0.50,  0.52, 0.38,
            -0.27, -0.415,  1.05,   -0.27,  -0.745, 0.65,
    })*DEG_TO_RAD

--Hip roll change
  walk.legBias = 
    vector.new({1, 1.26,    0.36,   -0.50,  0.52, 0.38,
            -0.27, -0.8875,  1.05,   -0.27,  -0.745, 0.65,
    })*DEG_TO_RAD

--pitch tweak

  walk.legBias = 
    vector.new({1, 1.26,    0.36,   -0.50,  0.52, 0.38,
            -0.27, -0.8875,  1.05,   -0.27,  -0.745, 0.65,
    })*DEG_TO_RAD

--[[
--new roll tweak
  walk.legBias = 
    vector.new({1, 1.26,    0.36,   -0.23,  0.52, 0.38,
            -0.27, -0.2875,  1.05,   -0.27,  -0.745, 0.65,
    })*DEG_TO_RAD
--]]


--NRL site, 1/30/2014

  walk.legBias = 
    vector.new({1, 1.26,    0.36,   -0.50,  0.72, 0.38,
            -0.27, -0.8875,  1.05,   -0.27,  -0.205, 0.65,
    })*DEG_TO_RAD



--slight hip roll tweak
  walk.legBias = 
    vector.new({1, 1.26,    0.36,   -0.50,  0.72, 0.38,
            -0.27, -1.3,  1.05,   -0.27,  -0.205, 0.65,
    })*DEG_TO_RAD




--after power shutdown
  walk.legBias = 
    vector.new({1, 1.26, 0.36,   -0.365,  0.72, 0.38,
            0.47, -1.3, 1.05,   -0.88,  -0.20, -0.16,
    })*DEG_TO_RAD


  walk.legBias = 
    vector.new({1, 1.19, 0.36,   -0.365,  -0.29, -0.02,
            0.47, -1.3, 1.05,   -0.88,  -1.4, -0.51,
    })*DEG_TO_RAD


--emergency fix
  walk.legBias = 
    vector.new({1, 0.72, 0.36,   -0.365,  -0.29, 0.45,
            0.47, -0.49, 1.05,   -0.88,  -1.4, 0.03,
    })*DEG_TO_RAD


--1.00 0.72 0.36 -0.36 -0.02 1.33
-- 0.47 -0.69 1.19 -0.88 -1.06 0.03 

  walk.legBias = 
    vector.new({1, 0.72, 0.36,   -0.365,  -0.02, 1.33,
            0.47, -0.69, 1.19,   -0.88,  -1.06, 0.03,
    })*DEG_TO_RAD



  walk.legBias = 
    vector.new({1, 0.72, 0.36,   -0.365,  0.32, 0.72,
            0.47, -0.69, 1.19,   -0.88,  -1.06, 0.03,
    })*DEG_TO_RAD





--NRL D-0, 7:40AM

--1.00 0.72 0.02 -0.36 0.32 0.72
--0.47 -0.69 1.19 -0.88 -0.66 0.03 


  walk.legBias = 
    vector.new({1, 0.72, 0.02,   -0.365,  0.32, 0.72,
            0.47, -0.69, 1.19,   -0.88,  -0.66, 0.03,
    })*DEG_TO_RAD



else
  print("UNKNOWN ROBOT")
  walk.legBias = 
    vector.new({0,0,0,0,0,0,
        0,0,0,0,0,0,
    })*DEG_TO_RAD
end










-----------------------------------------------------------
-- Stance parameters
-----------------------------------------------------------

local stance={}

--TEMPORARY HACK FOR PERCEPTION TESTING
--walk.bodyTilt = 0*math.pi/180

--Should we move torso back for compensation?
stance.enable_torso_compensation = 1
--stance.enable_torso_compensation = 0

stance.enable_sit = false
stance.enable_legs = true   -- centaur has no legs
stance.qWaist = vector.zeros(2)

stance.dqWaistLimit = 10*DEG_TO_RAD*vector.ones(2)
stance.dpLimitStance = vector.new{.04, .03, .03, .4, .4, .4}
stance.dqLegLimit = vector.new{10,10,45,90,45,10}*DEG_TO_RAD

stance.sitHeight = 0.75
stance.dHeight = 0.01 --4cm per sec


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
-- Associate with the table
Config.walk    = walk
Config.stance  = stance
Config.zmpstep = zmpstep

return Config
