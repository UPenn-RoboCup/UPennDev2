local vector = require'vector'
local DEG_TO_RAD = math.pi/180

local Config = {}

------------------------------------
-- For the arm FSM
local arm = {}


--Gripper end position offsets (Y is inside)
arm.handoffset={}
arm.handoffset.gripper = {0.245,0.035,0} --Default gripper
arm.handoffset.outerhook = {0.285,-0.065,0} --Single hook (for door)
arm.handoffset.chopstick = {0.285,0,0} --Two rod (for valve)

--Arm planner variables
arm.plan={}
arm.plan.max_margin = math.pi/6
--arm.plan.max_margin = math.pi
--arm.plan.dt_step0 = 0.5
--arm.plan.dt_step = 0.5
arm.plan.dt_step0 = 0.1
arm.plan.dt_step = 0.2
arm.plan.search_step = 1
--arm.plan.search_step = .25

arm.plan.velWrist = {100000,100000,100000, 15*DEG_TO_RAD,15*DEG_TO_RAD,15*DEG_TO_RAD}
arm.plan.velDoorRoll = 10*DEG_TO_RAD
arm.plan.velDoorYaw = 2*DEG_TO_RAD
--arm.plan.velTorsoComp = {0.005,0.005} --5mm per sec
arm.plan.velTorsoComp = {0.02,0.01} --5mm per sec
arm.plan.velYaw = 10*math.pi/180




-- Arm speed limits

arm.slow_limit = vector.new({10,10,10,15,30,30,30})*DEG_TO_RAD
arm.slow_elbow_limit = vector.new({10,10,10,5,30,30,30})*DEG_TO_RAD --Used for armInit

-- Linear movement speed limits
arm.linear_slow_limit = vector.new({0.02,0.02,0.02, 15*DEG_TO_RAD,15*DEG_TO_RAD,15*DEG_TO_RAD})

-- Use this for wrist initialization
arm.joint_init_limit=vector.new({30,30,30,30,30,30,30}) *DEG_TO_RAD

-- Use this for planned arm movement
arm.joint_vel_limit_plan = vector.new({10,10,10,10,30,10,30}) *DEG_TO_RAD

arm.linear_wrist_limit = 0.05


--Pose 1 wrist position
arm.pLWristTarget1 = {-.0,.30,-.20,0,0,0}
arm.pRWristTarget1 = {-.0,-.30,-.20,0,0,0}

arm.lShoulderYawTarget1 = -5*DEG_TO_RAD
arm.rShoulderYawTarget1 = 5*DEG_TO_RAD

arm.qLArmPose1 = vector.new({118.96025904076,9.0742631178663,-5,-81.120944928286,81,14.999999999986, 9})*DEG_TO_RAD
arm.qRArmPose1 = vector.new({118.96025904076,-9.0742631178663,5,-81.120944928286,-81,-14.999999999986, 9})*DEG_TO_RAD



if IS_WEBOTS then
  --Faster limit for webots
  local speed_factor = 5
  arm.slow_limit = arm.slow_limit*speed_factor
  arm.slow_elbow_limit = arm.slow_elbow_limit*speed_factor
  arm.linear_slow_limit = arm.linear_slow_limit*speed_factor
  arm.joint_init_limit=arm.joint_init_limit*speed_factor
  arm.joint_vel_limit_plan = arm.joint_vel_limit_plan*speed_factor
end


--Arm State specific infos
armfsm = {}


--Drill pickup 

armfsm.toolgrip = {}
armfsm.toolgrip.lhand_rpy = {0,0*DEG_TO_RAD, -45*DEG_TO_RAD}
armfsm.toolgrip.rhand_rpy = {0,0*DEG_TO_RAD, 45*DEG_TO_RAD}


--All for RIGHT arm
armfsm.toolgrip.arminit={
  {0.25,-0.20,-0.05},  
  {0.30,-0.20,-0.10},
  {0.35,-0.20,0.05},
}
armfsm.toolgrip.armhold={ 
  {0.20,0.0,-0.10},  
}
armfsm.toolgrip.tool_clearance={-0.08,0,0}
armfsm.toolgrip.tool_liftup = {0,0,0.05}
armfsm.toolgrip.tool_liftuppull = {-0.20,0,0.05}




armfsm.toolchop = {}
armfsm.toolchop.arminit={
  {0.25,-0.20,-0.05},  
  {0.30,-0.0,-0.10},
  {0.35,-0.0,0.0},
}



armfsm.toolchop.drill_clearance = {-0.05,0,0}




------------------------------------
-- Associate with the table
Config.walk    = walk
Config.arm     = arm
Config.armfsm     = armfsm
Config.stance  = stance
Config.zmpstep = zmpstep

return Config
