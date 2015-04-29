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

----------------------------------------------------------------------
-- Param for robot
----------------------------------------------------------------------

------------------------------------
-- Stance and velocity limit values
------------------------------------
walk.stanceLimitX = {-0.30,0.30}
walk.stanceLimitY = {0.16,0.30}
walk.stanceLimitA = {-10*DEG_TO_RAD,30*DEG_TO_RAD}

walk.bodyHeight = 0.93
walk.footY = 0.095

--FOR torque testing only!!!
--if IS_WEBOTS then walk.footY = 0.072 end


walk.footX = 0
walk.bodyTilt = 0
walk.torsoX = 0.02     -- com-to-body-center offset

------------------------------------
-- Gait parameters
------------------------------------
walk.tStep = 0.80
walk.tZMP = 0.33
walk.stepHeight = 0.03
walk.phSingle = {0.15,0.85}
walk.phZmp = {0.15,0.85}
walk.phComp = {0.1,0.9}
walk.phCompSlope = 0.2
walk.supportX = 0.07 --With clown feet, good for forward walking
walk.supportY = 0.06

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
walk.dShift = DEG_TO_RAD*vector.new{30,30,30,30}

walk.hipRollCompensation = 2*DEG_TO_RAD

-----------------------------------
walk.velLimitX = {-.10,.15}
walk.velLimitY = {-.06,.06}
walk.velLimitA = {-.2,.2}
walk.velDelta  = {0.025,0.02,0.1}
walk.foot_traj = 1 --curved step

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



--Load robot specific configs
local c = require'calibration'
if c.cal[HOSTNAME].legBias then walk.legBias = c.cal[HOSTNAME].legBias end
if c.cal[HOSTNAME].headBias then walk.headBias = c.cal[HOSTNAME].headBias end

if IS_WEBOTS then
  Config.supportY_preview = -0.02
  Config.supportY_preview2 = -0.01

  walk.supportY = 0.09
  --Higher COM walking test!
  walk.hipRollCompensation = 0*DEG_TO_RAD
  walk.ankleRollCompensation = 0*DEG_TO_RAD
  walk.bodyHeight = 0.93 --lower bodyheight again for rough terrain walk
  walk.tZMP = 0.345
  walk.velLimitX = {-.10,.30}
  walk.stepHeight = 0.050
end

if IS_WEBOTS then
  --WEBOTS

  walk.torsoX = -0.03     -- com-to-body-center offset
  walk.supportX = 0.03 --better
  walk.supportY = 0.05

  walk.kneePitchCompensation = 0*DEG_TO_RAD
  walk.footSagCompensation = {0.0,0.0}


--  walk.delay_threshold_angle = 2.5*math.pi/180
  walk.delay_threshold_angle = 999*math.pi/180 --disabled
  walk.delay_factor = {0.8,1.7}
  walk.velLimitX = {-.10,.20}


else
  walk.tZMP = 0.40
  walk.tStep = 0.80
  walk.dShift = {30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD}
  walk.hipRollCompensation = 1.5*DEG_TO_RAD
  walk.velLimitY = {-.06,.06}

  --quick fix with 7dof arm (lil slower)
  walk.velLimitX = {-.10,.10}
  walk.torsoX = 0.0     -- com-to-body-center offset
  walk.supportX = 0.07 --better
  Config.supportY_preview = -0.03
  Config.supportY_preview2 = -0.02
  walk.supportX = 0.05 --better
  walk.supportY = 0.05



--  walk.delay_threshold_angle = 2.5*math.pi/180
  walk.delay_threshold_angle = 999*math.pi/180 --disabled
  walk.delay_factor = {0.8,1.7}

end

walk.supportX = 0.01
walk.supportY = 0.04
walk.tStep = .75
walk.tZMP = 0.33
walk.stepHeight = 0.04
walk.phSingle = {0.2,0.8}
walk.phZmp = {0.25,0.75}
walk.phComp = {0.1,0.9}
walk.phCompSlope = 0.2
walk.supportX = 0.025
walk.supportY = 0.00

------------------------------------
-- Associate with the table
Config.walk    = walk
Config.kick  = kick
Config.stance  = stance

--Now we keep a lookup table file rather than specifying here
local zparam = require'zmpparam'
Config.zmpparam = zparam.zmpparam



return Config
