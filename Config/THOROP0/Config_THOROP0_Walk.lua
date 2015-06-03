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
--if IS_WEBOTS then gyroFactorX,gyroFactorY=0,0 end
walk.ankleImuParamX={1, 0.9*gyroFactorX,  1*DEG_TO_RAD, 5*DEG_TO_RAD}
walk.kneeImuParamX= {1, -0.3*gyroFactorX,  1*DEG_TO_RAD, 5*DEG_TO_RAD}
walk.ankleImuParamY={1, 1.0*gyroFactorY,  1*DEG_TO_RAD, 5*DEG_TO_RAD}
walk.hipImuParamY  ={1, 0.5*gyroFactorY,  2*DEG_TO_RAD, 5*DEG_TO_RAD}

--timing adjustment stabilization
walk.delay_threshold_angle = 2.5*math.pi/180
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


if HOSTNAME=="teddy2" or HOSTNAME=="dale" then 
--or Config.PLATFORM_NAME == "THOROP1" then -- or Config.PLATFORM_NAME = "THOROP1" then
  walk.delay_threshold_angle = 999*math.pi/180 --disabled
  walk.anklePitchLimit=vector.new{-40,40}*DEG_TO_RAD --teddy has ankle ROM limitation

walk.footY = 0.110 --wider


--  walk.tZMP = 0.33   
--  walk.footY = 0.115 --teddy, even wider
--  walk.supportX = 0.02 

  --for non birdwalk 
  walk.hipRollCompensation = {2.5*DEG_TO_RAD, 1.5*DEG_TO_RAD}
  walk.supportY = -0.01
  walk.delay_threshold_angle = 2.5*math.pi/180
  walk.stop_threshold_angle = 5*math.pi/180
else
  --CHIP CHIP CHIP CHiP
  print("CHIP CHIP CHIP")
  walk.velLimitX = {-.10,.20}
  walk.velLimitY = {-.06,.06}
  walk.delay_threshold_angle = 2.5*math.pi/180
  Config.supportY_preview = 0.0 --this smooths out first step a bit
  Config.supportY_preview2 = 0.0
  walk.supportY = 0.0
  walk.hipRollCompensation = {2.5*DEG_TO_RAD, 1.5*DEG_TO_RAD}
end
------------------------------------
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

return Config

