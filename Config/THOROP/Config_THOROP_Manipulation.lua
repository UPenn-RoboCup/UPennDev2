assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

Config.IS_LONGARM =true
--Config.IS_LONGARM =false

------------------------------------
-- For the arm FSM
local arm = {}

arm.default_hand_mass = 0.4



--Gripper end position offsets (Y is inside)
arm.handoffset={}

--0.130 + 0.60+0.50
--arm.handoffset.gripper = {0.241,0,0} --Default gripper
arm.handoffset.gripper = {0.23,0,0} --Default gripper (VT)
--0.130+0.139+0.80-0.10
arm.handoffset.outerhook = {0.339,0,0.060} --Single hook (for door)
--0.130 + 0.140+0.80-0.10
--arm.handoffset.chopstick = {0.340,0,0} --Two rod (for valve)

--FROM EMPIRICAL DATA
arm.handoffset.chopstick = {0.440,0,0} --Two rod (for valve)




--Torques for finger controls
arm.torque={}
arm.torque.movement = 5
arm.torque.open = -10
arm.torque.grip_hose = 10
arm.torque.grip_drill = 10
arm.torque.grip_drill_trigger1 = 40
arm.torque.grip_drill_trigger2 = 40




--Arm planner variables
arm.plan={}
arm.plan.max_margin = math.pi/6
arm.plan.dt_step0 = 0.1
arm.plan.dt_step = 0.2
arm.plan.search_step = 1
--arm.plan.max_margin = math.pi/2
arm.plan.search_step = 0.25




-- Arm speed limits
arm.shoulder_yaw_limit = 10*math.pi/180
arm.torso_comp_limit = vector.new({0.02,0.01})


--testing for speeding up
arm.overspeed_factor = 3
--arm.overspeed_factor = 1

arm.vel_linear_limit = vector.new({0.02,0.02,0.02, 30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD})
arm.vel_angular_limit = vector.new({20,20,20,20,30,30,30})*DEG_TO_RAD


--Servo limit
-- 30rpm 30rpm 30rpm 30rpm 20rpm 20rpm 20rpm
-- = 180 180 180 180 120 120 120 degree/sec

arm.shoulder_yaw_limit = 20*DEG_TO_RAD

local speed_factor = 1

------------------------------------------------------
--OLD LIMITS FOR WEBOTS
--arm.overspeed_factor = 1arm_planner:set_shoulder_yaw_target(qLArm0[3], nil) --Lock left hand
--arm.vel_linear_limit = vector.new({0.06,0.06,0.06, 30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD})
--arm.vel_angular_limit = vector.new({30,30,30,30,90,30,90})*DEG_TO_RAD
arm.shoulder_yaw_limit = 30*DEG_TO_RAD
arm.torso_comp_limit = vector.new({0.06,0.03})
------------------------------------------------------


--Pose 1 wrist position
arm.pLWristTarget0 = {-.0,0.25,-0.25,0,0,0}
arm.pRWristTarget0 = {-.0,-0.25,-0.25,0,0,0}

--POse 1 wrist angle
arm.lrpy0 = vector.new({0,0,0,0, 0,0})*DEG_TO_RAD
arm.rrpy0 = vector.new({0,0,0,-0,0,0})*DEG_TO_RAD

arm.lShoulderYaw0 = 5*DEG_TO_RAD
arm.rShoulderYaw0 = -5*DEG_TO_RAD

arm.qLArmPose1 = vector.new({118.96025904076,9.0742631178663,-5,-81.120944928286,81,14.999999999986, 9})*DEG_TO_RAD
arm.qRArmPose1 = vector.new({118.96025904076,-9.0742631178663,5,-81.120944928286,-81,-14.999999999986, 9})*DEG_TO_RAD

--Arm State specific infos
armfsm = {}



---------------------------------------------------------------
------   Drill pickup
---------------------------------------------------------------
armfsm.toolgrip = {}
--armfsm.toolgrip.lhand_rpy = {0,0*DEG_TO_RAD, -45*DEG_TO_RAD}
armfsm.toolgrip.lhand_rpy = {0,0*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.toolgrip.rhand_rpy = {0,0,45*DEG_TO_RAD}

--The optimal model
armfsm.toolgrip.default_model_target = {
  0.55,-0.06,0.20,  0*DEG_TO_RAD}    --xyz, yaw

--Conservative initial model (away from target)
armfsm.toolgrip.default_model = {
  0.42,-0.06,0.20,  0*DEG_TO_RAD}


armfsm.toolgrip.tool_clearance={-0.05,0,0}
armfsm.toolgrip.tool_liftup = {0,0,0.05}
armfsm.toolgrip.tool_clearance_x = 0.38

armfsm.toolgrip.turnUnit = 7.5*DEG_TO_RAD --Yaw

armfsm.toolgrip.armchecktrigger={
  {0.33,-0.25,-0.15, 0,0*DEG_TO_RAD, 90*DEG_TO_RAD},
  {0.46,-0.35,0.07,  0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
}





--Table height: 0.95
--Drill height: 0.13
--Hand height: 1.08 = 0.93 + 0.15

armfsm.toolgrip.arminit={
  {0.33,-0.25,-0.15, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.46,-0.35,0.07,  0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.46,-0.35, 0.15, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.46,-0.25, 0.15, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
}

armfsm.toolgrip.armpull={
  --0.55 -0.06 0.25
  {0.46, -0.25, 0.17, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.46, -0.35, 0.17, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.32, -0.35, -0.15, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
}


--Table height: 0.95
--Drill height: 0.13
--Hand height: 1.08 = 0.93 + 0.15

--New init sequence that avoids the table

armfsm.toolgrip.arminit={
  {0.29,-0.40,-0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.39,-0.40,0.01,0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.46,-0.40,0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.46,-0.40,0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
}

--for ik testing
armfsm.toolgrip.arminit={
  {0.29,-0.20,-0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.39,-0.20,0.01,0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.46,-0.20,0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.46,-0.20,0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
}







armfsm.toolgrip.armpull={
  {0.42,-0.40,0.20,0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.39,-0.40,0.01,0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.25,-0.40,-0.10,0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
}

armfsm.toolgrip.armhold={0.25,0.0,-0.20,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}

--FOR 2nd CAMERA
--Roll Pitch Yaw
armfsm.toolgrip.larm={
  {0.29,-0.40,-0.15,-45*DEG_TO_RAD,0*DEG_TO_RAD, -10*DEG_TO_RAD},
}


armfsm.toolgrip.default_model =
--{0.42,-0.06,0.15,  0*DEG_TO_RAD}
{0.42,-0.3,0.15,  0*DEG_TO_RAD}    --shoulder position






------------------------------------
-- Associate with the table
Config.arm     = arm
Config.armfsm  = armfsm

return Config
