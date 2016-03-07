assert(Config, 'Need a pre-existing Config table!')
--print("Robot hostname:",HOSTNAME)

--THOR mk2 specific walk config
--Only one robot (chipettes)
local vector = require'vector'

------------------------------------
-- Walk Parameters

local walk = {}
walk.legBias = vector.new({0,0,0,0,0,0,0,0,0,0,0,0,})*DEG_TO_RAD
walk.velocityBias = {0.0,0,0} --To get rid of drifting

-- Servo params
walk.init_command_velocity = 500
walk.init_command_accelleration = 50
walk.leg_p_gain = 64
walk.ankle_p_gain = 64

--Default-y vaue
walk.maxTurnSpeed = 0.1
walk.aTurnSpeed = 0.25
walk.maxStepCount = 30

------------------------------------
-- Stance and velocity limit values
------------------------------------
walk.stanceLimitX = {-0.30,0.30}
walk.stanceLimitY = {0.20,0.30} --mk2 pelvis width:0.210  
walk.stanceLimitA = {-0*DEG_TO_RAD,30*DEG_TO_RAD}
if Config.birdwalk then
  walk.stanceLimitA = {-30*DEG_TO_RAD,0*DEG_TO_RAD}
end
walk.bodyHeight = 0.93
walk.footY = 0.105 --mk2, wider
walk.footX = 0
walk.bodyTilt = 0
walk.torsoX = 0.02     -- com-to-body-center offset (which is for initial seed for numerical solver)
walk.torsoX = 0.0     -- com-to-body-center offset

------------------------------------
-- Gait parameters
------------------------------------
walk.tStep = 0.80
walk.tZMP = 0.32
walk.stepHeight = 0.04 
walk.phComp = {0.1,0.9}
walk.phSingle = {0.2,0.8}
walk.phZmp = {0.25,0.75}
walk.phCompSlope = 0.2
walk.supportX = 0.0 
walk.supportY = 0.0

--Config.supportY_preview = 0.01 -- support position for the first step
--Config.supportY_preview2 = 0.0 -- support position for preview-based steps

Config.supportY_preview = 0.0 --this smooths out first step a bit
Config.supportY_preview2 = 0.0  



------------------------------------
-- Compensation parameters
------------------------------------

gyroFactorX = 490.23/(251000/180)*0.5
gyroFactorY = 490.23/(251000/180)*0.5
if IS_WEBOTS then gyroFactorX,gyroFactorY=0,0 end
walk.ankleImuParamX={1, 0.9*gyroFactorX,  1*DEG_TO_RAD, 5*DEG_TO_RAD}
walk.kneeImuParamX= {1, -0.3*gyroFactorX,  1*DEG_TO_RAD, 5*DEG_TO_RAD}
walk.ankleImuParamY={1, 1.0*gyroFactorY,  1*DEG_TO_RAD, 5*DEG_TO_RAD}
walk.hipImuParamY  ={1, 0.5*gyroFactorY,  2*DEG_TO_RAD, 5*DEG_TO_RAD}

--timing adjustment stabilization
walk.delay_threshold_angle = 2.5*math.pi/180
walk.stop_threshold_angle = 5*math.pi/180
walk.delay_factor = {0.8,1.7}

walk.ankleRollCompensation = 0*DEG_TO_RAD  
walk.footSagCompensation = {0.0,0.0}
walk.kneePitchCompensation = 0*DEG_TO_RAD
walk.hipRollCompensation = 1.5*DEG_TO_RAD
    

-----------------------------------
walk.velLimitX = {-.10,.15}
walk.velLimitY = {-.06,.06}
walk.velLimitA = {-.2,.2}
walk.velDelta  = {0.025,0.02,0.1}

-----------------------------------------------------------
-- Stance parameters
-----------------------------------------------------------

local stance={}

stance.enable_torso_compensation = 1 --Should we move torso back for compensation?
stance.enable_sit = false
stance.enable_legs = true   -- centaur has no legs
stance.qWaist = vector.zeros(2)
stance.dqWaistLimit = 10*DEG_TO_RAD*vector.ones(2)
stance.dpLimitStance = vector.new{.04, .03, .03, .4, .4, .4}
stance.dqLegLimit = vector.new{10,10,45,90,45,10}*DEG_TO_RAD
stance.sitHeight = 0.75
stance.dHeight = 0.04 --4cm per sec

walk.COMoffsetBias = 0.00 --default COM offset 

if HOSTNAME=="teddy2" or HOSTNAME=="dale" then 
  walk.delay_threshold_angle = 999*math.pi/180 --disabled
  walk.anklePitchLimit=vector.new{-40,40}*DEG_TO_RAD --teddy has ankle ROM limitation

  walk.delay_threshold_angle = 2.5*math.pi/180
  walk.stop_threshold_angle = 4*math.pi/180

  walk.velLimitX = {-.10,.20}
  walk.velLimitY = {-.06,.06}

  Config.supportY_preview = 0.0 --this smooths out first step a bit
  Config.supportY_preview2 = 0.0
  walk.supportY = 0.0  

  walk.velocityBias = {0.0,0,0} --To get rid of drifting
  walk.hipRollCompensation = {1.7*DEG_TO_RAD, 1.5*DEG_TO_RAD}

  
  walk.stepHeight = 0.02 
  walk.supportY = 0.01
  
else
  --CHIP CHIP CHIP CHiP
  print("CHIP CHIP CHIP")

  walk.delay_threshold_angle = 999*math.pi/180 --disabled
  walk.anklePitchLimit=vector.new{-40,40}*DEG_TO_RAD --teddy has ankle ROM limitation

  walk.delay_threshold_angle = 2.5*math.pi/180
  walk.stop_threshold_angle = 4*math.pi/180


  walk.delay_threshold_angle = 5*math.pi/180
  walk.stop_threshold_angle = 99*math.pi/180


  walk.velLimitX = {-.10,.15}
  walk.velLimitY = {-.06,.06}
  Config.supportY_preview = 0.0 --this smooths out first step a bit
  Config.supportY_preview2 = 0.0
  walk.supportY = 0.0  
  walk.velocityBias = {0.0,0,0} --To get rid of drifting
  walk.hipRollCompensation = {1.7*DEG_TO_RAD, 1.5*DEG_TO_RAD}

  walk.stepHeight = 0.04 
  walk.tZMP = 0.30
  walk.supportY = -0.01

  walk.COMoffsetBias = 0.0 --default COM offset 
  walk.COMoffsetBias = -0.01 --default COM offset 

  walk.supportY = -0.0

  walk.stepHeight = 0.05 

end


--uneven terrain related---
walk.raise_body = false
walk.use_exact_tZMP = false 
-----------------------------

walk.traj={}
walk.traj.hybridwalk = "foot_trajectory_softfast"
walk.traj.hybridwalk = "foot_trajectory_base2"
walk.traj.hybridwalk = "foot_trajectory_base"--sin wave
--walk.traj.hybridwalk = "foot_trajectory_base3"

walk.variable_tstep = false
walk.variable_support = false
walk.use_heeltoe_walk = false
walk.heeltoe_angle = 0*DEG_TO_RAD


walk.use_heeltoe_walk = true
walk.heeltoe_angle = 5*DEG_TO_RAD
walk.heeltoe_angle = -5*DEG_TO_RAD
walk.heeltoe_angle = 0*DEG_TO_RAD

--walk.heeltoe_angle = 5*DEG_TO_RAD
walk.stepHeight = 0.03 



if IS_WEBOTS then
  walk.hipRollCompensation = {0*DEG_TO_RAD, 0*DEG_TO_RAD}
--  walk.traj.hybridwalk = "foot_trajectory_softfast"
--  walk.stepHeight = 0.06 

--  walk.COMoffsetBias = -0.03 --default COM offset 

  walk.phSingle = {0.2,0.8}
  walk.phZmp = {0.25,0.75}
  walk.phSingle = {0.15,0.85}
  walk.phZmp = {0.15,0.85}
  walk.velLimitX = {-.10,.30}
  walk.velLimitY = {-.06,.06}
  walk.velDelta  = {0.05,0.02,0.15}
  walk.velLimitX = {-.10,.60}
  walk.stanceLimitX = {-0.60,0.60}
  walk.stepHeight = 0.06
end

---------kinda works
walk.traj.hybridwalk = "foot_trajectory_base"--sin wave
walk.stepHeight = 0.03 
------------------------------


--kinda works #2
---------------------------------------------------------------
walk.phSingle = {0.2,0.8}
walk.phZmp = {0.2,0.8}
walk.stepHeight = 0.04 
walk.tZMP = 0.33
walk.hipRollCompensation = {1.7*DEG_TO_RAD, 1.7*DEG_TO_RAD}
walk.supportY = 0.01

--This helps a bit with hip pitch lag (with weird side effect)
walk.hipPitch0 = -29.21*DEG_TO_RAD
walk.hipPitchCompensationMag = 1.3


walk.hipPitch0 = nil --disable

----------------------------------------------------------------





walk.velLimitX = {-.10,.15}
walk.velLimitY = {-.05,.05}
walk.velDelta  = {0.05,0.02,0.1}

--sidestepping zmp position modulation
walk.sideModL = 0.03 
walk.sideMod2L = -0.01 
walk.sideModR = -0.03 
walk.sideMod2R = 0.01 



--DARWIN!
--------------------------------------------------------------------
walk.anklePitchLimit=vector.new{-90,90}*DEG_TO_RAD --teddy has ankle ROM limitation

walk.bodyHeight = 0.295
walk.footX=-0.020
walk.footY=0.035
walk.bodyTilt=20*math.pi/180; 
walk.bodyTilt=0*math.pi/180; 
walk.tZMP = 0.165

walk.tStep=0.4
walk.stanceLimitY={0.07,0.20};
walk.hipRollCompensation = {0*DEG_TO_RAD, 0*DEG_TO_RAD}

walk.minSitHeight = 0.10
stance.dHeight = 0.005
stance.sitHeight = 0.10
stance.initHeight =0.30
walk.stepHeight = 0.035;
walk.use_heeltoe_walk = false

walk.supportY = 0.03
walk.sideModL = 0.00 
walk.sideMod2L = -0.00 
walk.sideModR = -0.00 
walk.sideMod2R = 0.00 


walk.phSingle = {0.15,0.85}
walk.phZmp = {0.15,0.85}
walk.tStep=0.6
walk.supportY = 0.015
walk.stanceLimitX = {-0.40,0.40}
walk.stepHeight = 0.035;


walk.velLimitX = {-.125,.175}
walk.stepHeight = 0.045;
walk.phSingle = {0.15,0.85}
walk.phZmp = {0.15,0.85}
walk.tStep=1.5

walk.delay_threshold_angle = 125*math.pi/180
walk.stop_threshold_angle = 125*math.pi/180

--------------------------------------------------------------------
--works till here



--forced heel-toe test
use_heeltoe_walk=false
walk.heel_angle,walk.toe_angle = 0,0

--use_heeltoe_walk=true
--walk.heel_angle = 30*DEG_TO_RAD
--walk.toe_angle = 10*DEG_TO_RAD

use_heeltoe_walk=true
walk.toe_angle = 30*DEG_TO_RAD
walk.heel_angle = 30*DEG_TO_RAD
walk.heeltoe_vel_min = 0.08

--ROlling zmp
walk.zmpHeel,walk.zmpToe = -0.03,0.03
walk.zmpHeel,walk.zmpToe = 0,0


walk.zmpHeel,walk.zmpToe = -0.0,0.05


--[[
walk.tStep=0.8
walk.supportY = 0.03
--]]

walk.tStep=1.2
walk.velLimitX = {-.125,.18} --max vel with heeltoe
 
--walk.velLimitX = {-.125,.08} --max vel without heeltoe, 295mm bodyheight
------------------------------------

--[[
walk.bodyHeight = 0.24
--]]

--[[
walk.bodyHeight = 0.24
walk.velLimitX = {-.125,.12} --max vel with heeltoe
walk.tStep=2
--]]








walk.zmpHeel,walk.zmpToe = -0.0,0.0
walk.velLimitX = {-.125,.15} --max vel with heeltoe

walk.bodyTilt = 0*math.pi/180




use_heeltoe_walk=true
walk.toe_angle = 0*DEG_TO_RAD
walk.heel_angle = 0*DEG_TO_RAD
walk.heeltoe_vel_min = 0.08

walk.velLimitX = {-.125,.12} --max vel with heeltoe
walk.tStep=1 --this works with robot
walk.testT={4.5,6.0} --one step
walk.testT={5.5,7.0} --two step


walk.use_heeltoe_walk = true
walk.toe_angle = 20*DEG_TO_RAD
walk.heel_angle = 20*DEG_TO_RAD
walk.heeltoe_vel_min = 0.05


walk.testT={10,11} --two step
walk.velLimitX = {-.125,.18} --max vel with heeltoe
walk.velDelta  = {0.04,0.02,0.1}
walk.zmpHeel,walk.zmpToe = -0.01,0.02
walk.heeltoe_vel_min = 0.10
walk.heeltoe_vel_min_zmp = 0.10
walk.heeltoe_vel_min = 0.10
walk.heeltoe_vel_min_zmp = 0.12
walk.heel_angle = 25*DEG_TO_RAD



--for video
--[[
walk.velLimitX = {-.125,.12} --max vel with heeltoe
walk.bodyHeight=0.245
walk.heeltoe_vel_min = 0.13
--]]

--
--------------------STEP UPDOWN TESTING
walk.stepHeightStairMin = 0.02
walk.stepHeightStairMax = 0.07
walk.stepStairst = 1.0
walk.stepStairwt = 3.0

walk.anklePitchLimit=vector.new{-90,90}*DEG_TO_RAD --teddy has ankle ROM limitation
walk.raise_body = true
walk.use_exact_tZMP = true
--------------------------------------------
--]]


--[[

--ROBOT
walk.testT={5.5,7.0} --two step
walk.velLimitX = {-.125,.12} --max vel with heeltoe
walk.velDelta  = {0.06,0.02,0.1}
walk.zmpHeel,walk.zmpToe = -0.0,0.0
walk.heeltoe_vel_min = 0.08

--]]

-- Associate with the table
Config.walk    = walk
Config.stance  = stance

--Load robot specific configs
local c = require'calibration'
if c.cal[HOSTNAME].legBias then walk.legBias = c.cal[HOSTNAME].legBias end
if c.cal[HOSTNAME].headBias then walk.headBias = c.cal[HOSTNAME].headBias end

--Now we keep a lookup table file rather than specifying here
local zparam = require'zmpparam'
Config.zmpparam = zparam.zmpparam
--Config.kick = kick

return Config
