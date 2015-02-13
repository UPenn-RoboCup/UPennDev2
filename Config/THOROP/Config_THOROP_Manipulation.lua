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

arm.vel_linear_limit = vector.new({0.06,0.06,0.06, 30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD})
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
arm.pLWristTarget0 = {-.0,.30,-.20,0,0,0}
arm.pRWristTarget0 = {-.0,-.30,-.20,0,0,0}

--ONLY FOR GETUP!!!
--
arm.pLWristTarget0 = {-.10,.30,-.20,0,0,0}
arm.pRWristTarget0 = {-.10,-.30,-.20,0,0,0}
--]]


--POse 1 wrist angle
arm.lrpy0 = vector.new({0,0,0,0,30,0})*DEG_TO_RAD
arm.rrpy0 = vector.new({0,0,0,-0,30,0})*DEG_TO_RAD

--[[
--New angle
--POse 1 wrist angle
arm.lrpy0 = vector.new({0,0,0,0,5,-45})*DEG_TO_RAD
arm.rrpy0 = vector.new({0,0,0,-0,5,45})*DEG_TO_RAD
--]]





--[[
--now hand facing down
arm.lrpy0 = vector.new({0,0,0,0,80,0})*DEG_TO_RAD
arm.rrpy0 = vector.new({0,0,0,0,80,0})*DEG_TO_RAD
--]]


arm.lShoulderYaw0 = 5*DEG_TO_RAD
arm.rShoulderYaw0 = -5*DEG_TO_RAD

arm.qLArmPose1 = vector.new({118.96025904076,9.0742631178663,-5,-81.120944928286,81,14.999999999986, 9})*DEG_TO_RAD
arm.qRArmPose1 = vector.new({118.96025904076,-9.0742631178663,5,-81.120944928286,-81,-14.999999999986, 9})*DEG_TO_RAD

--Arm State specific infos
armfsm = {}


---------------------------------------------------------------
------   Debris pickup
---------------------------------------------------------------
armfsm.debrisgrip = {}
armfsm.debrisgrip.lhand_rpy = {0,0*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.debrisgrip.rhand_rpy = {90*DEG_TO_RAD,45*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.debrisgrip.rhand_rpy = {90*DEG_TO_RAD,45*DEG_TO_RAD, 0*DEG_TO_RAD}

armfsm.debrisgrip.lhand_rpy = {0,0*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.debrisgrip.rhand_rpy = {0*DEG_TO_RAD,89*DEG_TO_RAD, 0*DEG_TO_RAD}




armfsm.debrisgrip.body_bend = 30*DEG_TO_RAD

--armfsm.debrisgrip.body_bend = 45*DEG_TO_RAD

--xyz, yaw
armfsm.debrisgrip.default_model = {
  0.30,-0.40,-0.20,  0*DEG_TO_RAD}

armfsm.debrisgrip.arminit={
---0.02 -0.30 -0.44
  {0.15,-0.30,-0.40, unpack(armfsm.debrisgrip.rhand_rpy)},

}

armfsm.debrisgrip.armpull={
  {0.38,-0.10,0.07,  0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.38,-0.10,0.07,  0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.33,-0.10,-0.20, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
}

armfsm.debrisgrip.clearance={0,0.08,0}
armfsm.debrisgrip.liftup = {0,0,0.05}

armfsm.debrisgrip.lhand_rpy = {0*DEG_TO_RAD,45*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.debrisgrip.rhand_rpy = {0*DEG_TO_RAD,45*DEG_TO_RAD, 0*DEG_TO_RAD}


--Take two
armfsm.debrisgrip.arminit={
---0.20 -0.30 -0.32  (0 30 0)
  {0.20,-0.40,-0.40, 0*DEG_TO_RAD,30*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.30,-0.40,-0.20, -90*DEG_TO_RAD,85*DEG_TO_RAD, 0*DEG_TO_RAD},
}

--Take three
armfsm.debrisgrip.arminit={
---0.20 -0.30 -0.32  (0 30 0)
  {0.40,-0.40,-0.30, 0*DEG_TO_RAD,30*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.40,-0.40,-0.20, -90*DEG_TO_RAD,85*DEG_TO_RAD, 0*DEG_TO_RAD},
}


--LEFT
--Take three
armfsm.debrisgrip.larminit={
---0.20 -0.30 -0.32  (0 30 0)
  {0.40,0.40,-0.30, 0*DEG_TO_RAD,30*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.40,0.40,-0.20, -90*DEG_TO_RAD,85*DEG_TO_RAD, 0*DEG_TO_RAD},
}


--WITH wolverine setup



--Take three
armfsm.debrisgrip.arminit={
---0.20 -0.30 -0.32  (0 30 0)
  {0.40,-0.40,-0.30, 0*DEG_TO_RAD,30*DEG_TO_RAD, 0*DEG_TO_RAD},
--  {0.40,-0.40,-0.20, 0*DEG_TO_RAD,85*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.40,-0.40,-0.10, 0*DEG_TO_RAD,85*DEG_TO_RAD, 0*DEG_TO_RAD},

--for returning
  {0.22, -0.40, -0.41, 0.0, 85*DEG_TO_RAD, 0}
}

--LEFT
armfsm.debrisgrip.larminit={
---0.20 -0.30 -0.32  (0 30 0)
  {0.40,0.40,-0.30, 0*DEG_TO_RAD,30*DEG_TO_RAD, 0*DEG_TO_RAD},
--  {0.40,0.40,-0.20, 0*DEG_TO_RAD,85*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.40,0.40,-0.10, 0*DEG_TO_RAD,85*DEG_TO_RAD, 0*DEG_TO_RAD},

  --for returning
  {0.22, 0.40, -0.41, 0.0, 85*DEG_TO_RAD, 0}
}






---------------------------------------------------------------
-- Open door sideways
---------------------------------------------------------------
armfsm.doorpushside = {}

armfsm.doorpushside.larminit = {
  {0.32,0.26,-0.05,  0,-15*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.22,0.26,-0.03,  0,-15*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.22,0.26,-0.03,  0,-15*DEG_TO_RAD, 90*DEG_TO_RAD},
--going back
  {0.0,0.64,-0.14,  0,-15*DEG_TO_RAD, 0*DEG_TO_RAD}, --last tr
}

armfsm.doorpushside.rarminit = {
  {0.32,-0.30,-0.05,  0,-15*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.47,-0.30,-0.03,  0,-15*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.47,-0.30,-0.03,  0,-15*DEG_TO_RAD, 0*DEG_TO_RAD},
--going back
  {0.0,0.64,-0.14,  0,-15*DEG_TO_RAD, 0*DEG_TO_RAD}, --last tr
}


--new rarm pos



armfsm.doorpushside.rarminit = {
  {0.32,-0.30,-0.05,  0,75*DEG_TO_RAD, 0*DEG_TO_RAD},
  --0.15 -0.30 -0.51
  {0.20, -0.30, -0.51,  0,75*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.20, -0.30, -0.51,  0,75*DEG_TO_RAD, 0*DEG_TO_RAD},
--going back
  {0.0,0.64,-0.14,  0,755*DEG_TO_RAD, 0*DEG_TO_RAD}, --last tr
}

--FOR CAMERA

armfsm.doorpushside.rarminit = {
  {0.32,-0.30,-0.05,  0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  --0.24 -0.06 -0.14
  {0.32, -0.15, -0.14,  0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.32, -0.15, -0.14,  0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
--going back
  {0.0,0.64,-0.14,  0,75*DEG_TO_RAD, 0*DEG_TO_RAD}, --last tr
}







armfsm.doorpushside.bodyyaw = 45*DEG_TO_RAD








armfsm.doorpushside.unit_tilt = 5*DEG_TO_RAD
armfsm.doorpushside.unit_yaw = 5*DEG_TO_RAD













armfsm.doorpullside = {}

armfsm.doorpullside.larminit = {
 --0.32 0.30 -0.32 (0 30 0)
  {0.15,0.30,-0.32,  0,30*DEG_TO_RAD, 0*DEG_TO_RAD},
}

armfsm.doorpullside.rarminit = {
  --0.32 -0.30 -0.32 (0 30 0)
  {0.42,-0.30,-0.32,  0,30*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.38,-0.10,0.07,  0,0*DEG_TO_RAD, 90*DEG_TO_RAD},

--going back
  {0.0,0.64,-0.14,  0,0*DEG_TO_RAD, 90*DEG_TO_RAD}, --last tr
}

--little tilt angle up

armfsm.doorpullside.unit_tilt = 5*DEG_TO_RAD
armfsm.doorpullside.unit_yaw = 2*DEG_TO_RAD



















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

--[[
---------------------------------
--WIth hand yaw control
---------------------------------

armfsm.toolgrip.arminit={
  {0.31,-0.50,-0.15,0,0*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.45,-0.58,0.01,0,0*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.50,-0.58, 0.17,0,0*DEG_TO_RAD, 0*DEG_TO_RAD},

  {0.46,-0.40,0.17,0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
}


armfsm.toolgrip.armpull={
  {0.42,-0.40, 0.25,0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.50,-0.53, 0.17,0,0*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.45,-0.53,0.01,0,0*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.31,-0.50,-0.15,0,0*DEG_TO_RAD, 0*DEG_TO_RAD},
}
--]]












armfsm.toolleftgrip={}
armfsm.toolleftgrip.lhand_rpy = {0,0*DEG_TO_RAD, -45*DEG_TO_RAD}
armfsm.toolleftgrip.rhand_rpy = {0,0,0*DEG_TO_RAD}
--The optimal model
armfsm.toolleftgrip.default_model_target = {
  0.55,0.06,0.20,  0*DEG_TO_RAD}    --xyz, yaw

--Conservative initial model (away from target)
armfsm.toolleftgrip.default_model = {
  0.42,0.06,0.20,  0*DEG_TO_RAD}

armfsm.toolleftgrip.arminit={
  {0.33,0.25,-0.15, 0,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  {0.46,0.35,0.07,  0,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  {0.46,0.35, 0.17, 0,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  {0.46,0.25, 0.17, 0,0*DEG_TO_RAD, -45*DEG_TO_RAD},
}

armfsm.toolleftgrip.armpull={
  --0.55 -0.06 0.25
  {0.46, 0.25, 0.17, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.46, 0.35, 0.17, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.32, 0.35, -0.15, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
}








---------------------------------------------------------------
------   Drill cut
---------------------------------------------------------------
armfsm.toolchop = {}
--How much torso should follow arm?
armfsm.toolchop.torsoMovementMag = 0.5
armfsm.toolchop.drill_clearance = {-0.05,0,0}
armfsm.toolchop.drill_offset = {0,0,0.10}

--armfsm.toolgrip.armhold={0.25,0.0,-0.20}

armfsm.toolchop.arminit={
  {0.25,-0.16,-0.20},
  {0.25,-0.16,-0.20},
  {0.30,-0.16,-0.07},
}

armfsm.toolchop.model={0.30,-0.16,0.03,0}

---------------------------------------------------------------
------   Hose pickup
---------------------------------------------------------------

armfsm.hosegrip = {}
armfsm.hosegrip.lhand_rpy = {0,0*DEG_TO_RAD, -45*DEG_TO_RAD}
armfsm.hosegrip.rhand_rpy = {0,0*DEG_TO_RAD, 45*DEG_TO_RAD}

armfsm.hosegrip.lhand_rpy1 = {135*DEG_TO_RAD,0,-45*DEG_TO_RAD}
armfsm.hosegrip.rhand_rpy1 = {-135*DEG_TO_RAD,0,45*DEG_TO_RAD}

armfsm.hosegrip.lhand_rpy2 = {135*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.hosegrip.rhand_rpy2 = {-135*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD}



armfsm.hosegrip.armhold = {
  {0.33,0.10,-0.15},
  {0.28,0.35,-0.20},
}
armfsm.hosegrip.clearance={-0.06,0,0}
armfsm.hosegrip.bottompull={0,0,-0.06}


--with 45 deg
armfsm.hosegrip.default_model = {
  0.38,0.24, 0.025, 0, 0}

--REverted back, view blocked
armfsm.hosegrip.lhand_rpy = {0*DEG_TO_RAD,0*DEG_TO_RAD, -45*DEG_TO_RAD}
armfsm.hosegrip.lhand_rpy1 = {135*DEG_TO_RAD,0,-45*DEG_TO_RAD}
armfsm.hosegrip.lhand_rpy2 = {135*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD}

--Hose grip height: 37.5in or 0.93 + 0.025


--0.16 0.14 -0.20 (0 0 -45)
armfsm.hosegrip.arminit={
  {0.30,0.24,-0.15,0*DEG_TO_RAD,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  {0.38,0.24,0.025,0*DEG_TO_RAD,0*DEG_TO_RAD, -45*DEG_TO_RAD},
--0.16 0. -0.20 (179,0,45)
}

--RIGHT ARM (FOR CAMERA SUPPORT)
--init pose: 0.20 -0.30
armfsm.hosegrip.arminit_support={
--  {0.30,-0.24,-0.15,0*DEG_TO_RAD,0*DEG_TO_RAD, 45*DEG_TO_RAD},
--  {0.38,-0.24,0.025,0*DEG_TO_RAD,0*DEG_TO_RAD, 45*DEG_TO_RAD},

--wider pose
  {0.30,-0.30,-0.15,0*DEG_TO_RAD,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.45,-0.35,0.025,0*DEG_TO_RAD,0*DEG_TO_RAD, 45*DEG_TO_RAD},

--holding pose
  {0.20,-0.14,-0.20,0*DEG_TO_RAD,0*DEG_TO_RAD, 45*DEG_TO_RAD},
}


armfsm.hosegrip.armhold = {
  {0.33,0.35,-0.20,135*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD},
  {0.23,0.25,-0.20,135*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD},

  -- 0.16 0.14 -0.20 (0 0 -45)
  {-0.25,0.30,-0.30,135*DEG_TO_RAD,89*DEG_TO_RAD, 0*DEG_TO_RAD},
  {-0.30,0.05,-0.30,135*DEG_TO_RAD,89*DEG_TO_RAD, 0*DEG_TO_RAD},
}

armfsm.hosegrip.armhosepull={
  {-0.30,0.05,-0.30,135*DEG_TO_RAD,89*DEG_TO_RAD, 0*DEG_TO_RAD},
  {-0.25,0.30,-0.30,135*DEG_TO_RAD,89*DEG_TO_RAD, 0*DEG_TO_RAD},
  {-0.0,0.35,-0.43,135*DEG_TO_RAD,89*DEG_TO_RAD, 0*DEG_TO_RAD},
--  {0.23,0.35,-0.20,179*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD},

  {0.43,0.35,-0.20,135*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD},
}

armfsm.hosegrip.armhoseattachinit={
  {0.43,0.35,-0.20,135*DEG_TO_RAD,0*DEG_TO_RAD, -45*DEG_TO_RAD},
--  {0.43,0.35,-0.20,90*DEG_TO_RAD,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  {0.43,0.15,0.0,90*DEG_TO_RAD,0*DEG_TO_RAD, -45*DEG_TO_RAD},
}



--more away from the robot
armfsm.hosegrip.armhosepull={
  {-0.30,0.05,-0.30,135*DEG_TO_RAD,89*DEG_TO_RAD, 0*DEG_TO_RAD},
  {-0.25,0.40,-0.30,135*DEG_TO_RAD,89*DEG_TO_RAD, 0*DEG_TO_RAD},
  {-0.0,0.45,-0.43,135*DEG_TO_RAD,89*DEG_TO_RAD, 0*DEG_TO_RAD},
--  {0.23,0.35,-0.20,179*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD},

  {0.43,0.40,-0.20,135*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD},
}











---------------------------------------------------------------
------   Hose tap
---------------------------------------------------------------

armfsm.hosetap = {}
armfsm.hosetap.lhand_rpy0 = {}
armfsm.hosetap.lhand_rpy1 = {}

armfsm.hosetap.larminit={
  {0.21,0.18,-0.20, 90*DEG_TO_RAD,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  --0.21 0.19 -0.20

  {0.33,0.25,-0.15, 90*DEG_TO_RAD,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  {0.46,0.35, 0.07,  90*DEG_TO_RAD,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  {0.46,0.35, 0.17, 90*DEG_TO_RAD,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  {0.46,0.05, 0.17, 90*DEG_TO_RAD,0*DEG_TO_RAD, -45*DEG_TO_RAD},

}













---------------------------------------------------------------
------   Hose attach
---------------------------------------------------------------

armfsm.hoseattach = {}
armfsm.hoseattach.lhand_rpy0 = {90*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.hoseattach.rhand_rpy0 = {90*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.hoseattach.lhand_rpy1 = {90*DEG_TO_RAD,0*DEG_TO_RAD, -45*DEG_TO_RAD}
armfsm.hoseattach.rhand_rpy1 = {90*DEG_TO_RAD,0*DEG_TO_RAD, 45*DEG_TO_RAD}


armfsm.hoseattach.lhand_rpy = {90*DEG_TO_RAD,0*DEG_TO_RAD, -90*DEG_TO_RAD}
armfsm.hoseattach.rhand_rpy = {90*DEG_TO_RAD,0*DEG_TO_RAD, 90*DEG_TO_RAD}


armfsm.hoseattach.lhand_rpy = {90*DEG_TO_RAD,-15*DEG_TO_RAD, -90*DEG_TO_RAD}
armfsm.hoseattach.rhand_rpy = {90*DEG_TO_RAD,-15*DEG_TO_RAD, 90*DEG_TO_RAD}




armfsm.hoseattach.lhand_offset={-0.14,0,0}
armfsm.hoseattach.rhand_offset={-0.06,0,0}

armfsm.hoseattach.velTurnAngle = 6*DEG_TO_RAD
armfsm.hoseattach.velMove = 0.01

armfsm.hoseattach.angle1 = -45*DEG_TO_RAD
armfsm.hoseattach.angle2 = 45*DEG_TO_RAD


armfsm.hoseattach.rarm_clearance1 = 0.12
armfsm.hoseattach.rarm_clearance2 = 0.09

armfsm.hoseattach.larminit={
  {0.30,0.35,-0.20, unpack(armfsm.hoseattach.lhand_rpy0)},
  {0.35,0.18,-0.20, unpack(armfsm.hoseattach.lhand_rpy1)},
  {0.20,0.05,0.0, unpack(armfsm.hoseattach.lhand_rpy)},
}
armfsm.hoseattach.rarminit={
  {0.30,-0.35,-0.20, unpack(armfsm.hoseattach.rhand_rpy0)},
  {0.35,-0.18,-0.20, unpack(armfsm.hoseattach.rhand_rpy1)},
  {0.28,-0.05,0.0, unpack(armfsm.hoseattach.rhand_rpy)},
}

--Optimal model
--xyz yaw pitch
armfsm.hoseattach.default_model_target = {
  0.45,0.0, 0.10,   0,0}

--Conservative initial model, away from target
armfsm.hoseattach.default_model = {
  0.35,0.0, 0.10,   0,0}


---------------------------------------------------------------
------   Door pull open
---------------------------------------------------------------
armfsm.dooropen={}

--With top-mounted hook------------------------------------------------------
armfsm.dooropen.lhand_rpy={0*DEG_TO_RAD,-15*DEG_TO_RAD,-10*DEG_TO_RAD}
armfsm.dooropen.rhand_rpy={0*DEG_TO_RAD,-15*DEG_TO_RAD,-20*DEG_TO_RAD}

armfsm.dooropen.rhand_release={
--  {0,0,0,0*DEG_TO_RAD,-15*DEG_TO_RAD,-60*DEG_TO_RAD},
--  {0,0,0,0*DEG_TO_RAD,-15*DEG_TO_RAD,-60*DEG_TO_RAD},

  {0,0,0,0*DEG_TO_RAD,15*DEG_TO_RAD,-60*DEG_TO_RAD},
  {0,0,0,0*DEG_TO_RAD,15*DEG_TO_RAD,-60*DEG_TO_RAD},

  {0,0,0,0*DEG_TO_RAD,85*DEG_TO_RAD,-60*DEG_TO_RAD},
  {0,0,0,0*DEG_TO_RAD,85*DEG_TO_RAD,0*DEG_TO_RAD},
}

armfsm.dooropen.rhand_forward={
  {0,0,0,0*DEG_TO_RAD,85*DEG_TO_RAD, 20*DEG_TO_RAD},
  {0,0,0,0*DEG_TO_RAD,0*DEG_TO_RAD,20*DEG_TO_RAD}
}

armfsm.dooropen.rhand_sidepush={
  {0,0,0,0*DEG_TO_RAD,0*DEG_TO_RAD,-60*DEG_TO_RAD},
  {0,0,0,0*DEG_TO_RAD,60*DEG_TO_RAD,-60*DEG_TO_RAD},
  {0,0,0,0*DEG_TO_RAD,60*DEG_TO_RAD,0*DEG_TO_RAD}
}

--Hook up
armfsm.dooropen.handle_clearance0 = vector.new({-0.05,0,-0.05})
armfsm.dooropen.handle_clearance1 = vector.new({0,0,-0.05})
armfsm.dooropen.handle_clearance2 = vector.new({0,0,-0.05})

armfsm.dooropen.velDoorRoll = 10*DEG_TO_RAD * speed_factor
armfsm.dooropen.velDoorYaw = 2*DEG_TO_RAD * speed_factor




--Optimal model
armfsm.dooropen.default_model_target = {
  0.62,-1.16,0.01,  --Hinge pos
  0.84,             --Door width (hinge to knob axle)
  -0.05,            --Knob X offset from door
  -0.08,             --Knob Y offset (from knob axle)

  0,0,
}

--Conservative initial model
armfsm.dooropen.default_model = {
  0.45,-1.16,-0.07,  --Hinge pos
  0.84,             --Door width (hinge to knob axle)
  -0.05,            --Knob X offset from door
  -0.08,             --Knob Y offset (from knob axle)

  0,0, --Knob target roll and door target yaw
}



---------------------------------------------------------------
------   Loaded door pull open
---------------------------------------------------------------

armfsm.dooropen.lhand_support={
  {0,0,0,   0*DEG_TO_RAD,0*DEG_TO_RAD,-45*DEG_TO_RAD},
  --0.20 0.06 -0.16
  {0.45, 0.06, -0.06,   0*DEG_TO_RAD,0*DEG_TO_RAD,-45*DEG_TO_RAD},
  {0.45, 0.06, -0.06,   0*DEG_TO_RAD,0*DEG_TO_RAD,-90*DEG_TO_RAD},
  --0.16 -0.04 -0.06
  {0.35, -0.10, 0.04,   0*DEG_TO_RAD,0*DEG_TO_RAD,-90*DEG_TO_RAD},
  {0.40, -0.20, 0.15,   0*DEG_TO_RAD,0*DEG_TO_RAD,-90*DEG_TO_RAD},

  {0.30, -0.38, 0.15,   0*DEG_TO_RAD,0*DEG_TO_RAD,-90*DEG_TO_RAD},
}
armfsm.dooropen.lhand_unsupport={
  {0.20, -0.04, -0.16,   0*DEG_TO_RAD,0*DEG_TO_RAD,-90*DEG_TO_RAD},
  --0.44 0.06 -0.16
  {0.20, 0.06, -0.16,   0*DEG_TO_RAD,0*DEG_TO_RAD,-45*DEG_TO_RAD},
}


armfsm.dooropen.velWaistYaw = 3*DEG_TO_RAD * speed_factor
armfsm.dooropen.velWaistPitch = 3*DEG_TO_RAD * speed_factor
armfsm.dooropen.waistTarget = -15*DEG_TO_RAD

armfsm.dooropen.turnUnit = 15*DEG_TO_RAD --Knob roll
armfsm.dooropen.turnUnit2 = 2*DEG_TO_RAD --Door yaw

---------------------------------------------------------------
------   Door push open
---------------------------------------------------------------
armfsm.dooropenleft={}

armfsm.dooropenleft.lhand_rpy=
  {0,0,0,0*DEG_TO_RAD,-0*DEG_TO_RAD,-10*DEG_TO_RAD}

--optimal model
armfsm.dooropenleft.default_model_target = {
  0.52,-0.50,-0.04,  --Hinge pos
  0.84,             --Door width (hinge to knob axle)
  -0.05,            --Knob X offset from door
  -0.08,             --Knob Y offset (from knob axle)
  0,0 --knob target roll and door target yaw
}


--conservative initial model
armfsm.dooropenleft.default_model = {
  0.35,-0.50,-0.04,  --Hinge pos
  0.84,             --Door width (hinge to knob axle)
  -0.05,            --Knob X offset from door
  -0.08,             --Knob Y offset (from knob axle)
  0,0, --knob target roll and door target yaw
}

armfsm.dooropenleft.rhand_push={
  {0,0,0,0*DEG_TO_RAD,-0*DEG_TO_RAD, 45*DEG_TO_RAD},
--0.21 -0.06 -0.16
  {0.31, -0.0, -0.10,0*DEG_TO_RAD,-0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.45, -0.05, -0.0,0*DEG_TO_RAD,-0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.45, -0.05, -0.0,0*DEG_TO_RAD,-0*DEG_TO_RAD, -30*DEG_TO_RAD},
}

---------------------------------------------------------------
------   Circular valve turning with single hand w/chopstick
---------------------------------------------------------------
armfsm.valveonearm = {}

--Axel XYZ, radius, valve angle 1, valve angle 2

--height should be between 0.81(-0.12) to 1.22 (+0.29)
--boundary between low/med/high
armfsm.valveonearm.heights={0.03,0.15}



--with tilted init, should be used for bar grip as well
--[[
armfsm.valveonearm.arminit={
  {0.40,0.21,-0.15, 0,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  {0.55,0.28,0.07,  0,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  {0.60,0.01,0.07,  0,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  {0.72,0.30,0.09,  0,0*DEG_TO_RAD, 0*DEG_TO_RAD},
}
--]]
armfsm.valveonearm.turnUnit = 15*DEG_TO_RAD

armfsm.valveonearm.default_model_small=
  {0.72,0.30,0.09, 0, -60*DEG_TO_RAD, 60*DEG_TO_RAD }


armfsm.valveonearm.default_model_small=
  {0.67,0.30,0.09, 0, -60*DEG_TO_RAD, 60*DEG_TO_RAD }


--new init
armfsm.valveonearm.arminit={
  {0.40,0.25,-0.15, 0,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  {0.53,0.28,0.03,  0,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  {0.53,0.01,0.03,  0,0*DEG_TO_RAD, -45*DEG_TO_RAD},
  {0.67,0.30,0.03,  0,0*DEG_TO_RAD, 0*DEG_TO_RAD},
}


armfsm.valveonearm.rarminit={
  {0.40,-0.25,-0.15, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.53,-0.25,0.03,  0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.53,-0.25,0.03,  0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
}



armfsm.valveonearm.arminit_mirror={
  {0.40,-0.25,-0.15, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.53,-0.28,0.03,  0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.53,-0.01,0.03,  0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.67,-0.30,0.03,  0,0*DEG_TO_RAD, 0*DEG_TO_RAD},
}








armfsm.valveonearm.anglemargin = 5*DEG_TO_RAD
armfsm.valveonearm.clearance = -0.08
armfsm.valveonearm.velTurnAngle = 6*DEG_TO_RAD * speed_factor
armfsm.valveonearm.velInsert = 0.01 * speed_factor
---------------------------------------------------------------
------   Bar valve turning with single hand w/ chopstick
---------------------------------------------------------------
armfsm.valvebar = {}
--Axel XYZ, radius, current valve angle, final valve angle, hand fit angle,
--valve angle zero: straight UP

--armfsm.valvebar.handtightangle0 = -45*DEG_TO_RAD
armfsm.valvebar.handtightangle0 = -85*DEG_TO_RAD
armfsm.valvebar.clearance = -0.08

armfsm.valvebar.velTurnAngle = 6*DEG_TO_RAD * speed_factor
armfsm.valvebar.velInsert = 0.01 * speed_factor

--With updated chopstick IK


armfsm.valvebar.default_model= {0.73,0.25,0.09,
  0.05, 0, 0*DEG_TO_RAD, -10*DEG_TO_RAD  }


--RIGHT ARM
armfsm.valvebar.default_model_right= {0.73,-0.25,0.09,
  0.05, 0, 0*DEG_TO_RAD, -10*DEG_TO_RAD  }





armfsm.valvebar.arminit={{0.55,0.30,-0.15, 0,0,0}}


armfsm.valvebar.testAngle = 15*DEG_TO_RAD
armfsm.valvebar.turnUnit = 15*DEG_TO_RAD






---------------------------------------------------------------
------   Two-armed large valve turning (with chopstick)
---------------------------------------------------------------
armfsm.valvetwoarm = {0.55,0.20,0.07,   0.05, 0, 70*DEG_TO_RAD }

--Axel XYZ, radius, valve angle, hand fit angle
armfsm.valvetwoarm.velTurnAngle = 3*DEG_TO_RAD * speed_factor
armfsm.valvetwoarm.velInsert = 0.02 * speed_factor

--------------------------------------------------------------------------
---- Arm folding
--------------------------------------------------------------------------

armfsm.rocky={}

armfsm.rocky.lhand_rpy0 = {0*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.rocky.rhand_rpy0 = {0*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.rocky.lhand_rpy1 = {0*DEG_TO_RAD,0*DEG_TO_RAD, -45*DEG_TO_RAD}
armfsm.rocky.rhand_rpy1 = {0*DEG_TO_RAD,0*DEG_TO_RAD, 45*DEG_TO_RAD}
armfsm.rocky.lhand_rpy = {0*DEG_TO_RAD,-0*DEG_TO_RAD, -90*DEG_TO_RAD}
armfsm.rocky.rhand_rpy = {0*DEG_TO_RAD,-0*DEG_TO_RAD, 90*DEG_TO_RAD}


armfsm.rocky.larminit={
  {0.30,0.35,-0.20, unpack(armfsm.rocky.lhand_rpy0)},
  {0.40,0.18,-0.15, unpack(armfsm.rocky.lhand_rpy1)},
  --0.23 0.11 -0.15
  {0.23,0.11,-0.05, unpack(armfsm.rocky.lhand_rpy)},
}
armfsm.rocky.rarminit={
  {0.30,-0.35,-0.20, unpack(armfsm.rocky.rhand_rpy0)},
  {0.30,-0.18,-0.15, unpack(armfsm.rocky.rhand_rpy1)},
  --0.13 -0.11 -0.15
  {0.13,-0.11,-0.05, unpack(armfsm.rocky.rhand_rpy)},
}

armfsm.rocky.lhand_rpy0 = {0*DEG_TO_RAD,85*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.rocky.rhand_rpy0 = {0*DEG_TO_RAD,85*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.rocky.lhand_rpy1 = {0*DEG_TO_RAD,85*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.rocky.rhand_rpy1 = {0*DEG_TO_RAD,85*DEG_TO_RAD, 0*DEG_TO_RAD}

armfsm.rocky.lhand_rpy = {0*DEG_TO_RAD,85*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.rocky.rhand_rpy = {0*DEG_TO_RAD,85*DEG_TO_RAD, 0*DEG_TO_RAD}

armfsm.rocky.larminit={
  {0.20,0.30,-0.40, unpack(armfsm.rocky.lhand_rpy)},
  {0.30,0.18,-0.30, unpack(armfsm.rocky.lhand_rpy)},
  {0.25,-0.02,-0.35, unpack(armfsm.rocky.lhand_rpy)},

}
armfsm.rocky.rarminit={
  {-0.20,-0.30,-0.40, unpack(armfsm.rocky.rhand_rpy)},
  {-0.30,-0.18,-0.30, unpack(armfsm.rocky.rhand_rpy)},
--  {-0.30,-0.019,-0.30, unpack(armfsm.rocky.rhand_rpy)},

  {-0.30,-0.02,-0.301, unpack(armfsm.rocky.rhand_rpy)},
}


armfsm.doorpass={}
--armfsm.doorpass.lhand_rpy = {0*DEG_TO_RAD,0*DEG_TO_RAD, -15*DEG_TO_RAD}
--armfsm.doorpass.rhand_rpy = {0*DEG_TO_RAD,0*DEG_TO_RAD, 15*DEG_TO_RAD}

--default angle

armfsm.doorpass.lhand_rpy = {0*DEG_TO_RAD,30*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.doorpass.rhand_rpy = {0*DEG_TO_RAD,30*DEG_TO_RAD, 0*DEG_TO_RAD}

armfsm.doorpass.lhand_rpy2 = {0*DEG_TO_RAD,60*DEG_TO_RAD, 30*DEG_TO_RAD}
armfsm.doorpass.rhand_rpy2 = {0*DEG_TO_RAD,60*DEG_TO_RAD, -30*DEG_TO_RAD}

armfsm.doorpass.larminit={
--0.24 0.30 -0.20
  {0.14,0.27,-0.40, unpack(armfsm.doorpass.lhand_rpy)},
  {0.14,0.27,-0.40, unpack(armfsm.doorpass.lhand_rpy2)},
}
armfsm.doorpass.rarminit={
  {0.14,-0.27,-0.40, unpack(armfsm.doorpass.rhand_rpy)},
  {0.14,-0.27,-0.40, unpack(armfsm.doorpass.rhand_rpy2)},
}


--Take two, we use hook as feeler

armfsm.doorpass.lhand_rpy2 = {0*DEG_TO_RAD,60*DEG_TO_RAD, -30*DEG_TO_RAD}
armfsm.doorpass.rhand_rpy2 = {0*DEG_TO_RAD,60*DEG_TO_RAD, -30*DEG_TO_RAD}

armfsm.doorpass.lhand_rpy2 = {90*DEG_TO_RAD,45*DEG_TO_RAD, -0*DEG_TO_RAD}
armfsm.doorpass.rhand_rpy2 = {90*DEG_TO_RAD,45*DEG_TO_RAD, -0*DEG_TO_RAD}


armfsm.doorpass.larminit={
--0.24 0.30 -0.20
  {0.20,0.27,-0.40, unpack(armfsm.doorpass.lhand_rpy)},
  {0.20,0.27,-0.40, unpack(armfsm.doorpass.lhand_rpy2)},
}
armfsm.doorpass.rarminit={
  {0.20,-0.27,-0.40, unpack(armfsm.doorpass.rhand_rpy)},
  {0.20,-0.27,-0.40, unpack(armfsm.doorpass.rhand_rpy2)},
}



---------------------------------------------------------------
------ Fire Suppression (SAFFiR)
---------------------------------------------------------------
armfsm.firesuppress = {}
-- Default wrist angles
armfsm.firesuppress.lhand_rpy = {0,0,0}
armfsm.firesuppress.rhand_rpy = {0,0,0}

--Conservative initial model (away from target)
armfsm.firesuppress.default_model = {
  --0.55, -0.06, 0.1, 0, 45*DEG_TO_RAD}   --xyz,pitch,yaw
  0.4, -0.05, -0.1, 0, 45*DEG_TO_RAD}   --xyz,pitch,yaw

armfsm.firesuppress.turnUnit = 2.5 --degree

armfsm.firesuppress.arminit={  -- For right arm
  -- Gripper has a 45 degree Yaw
  {0.3,-0.25,-0.15, 0,0,0*DEG_TO_RAD},
  {0.35,-0.35, -0.1,  0,0,0*DEG_TO_RAD},
  {0.35,-0.35, -0.1, 0,0,0*DEG_TO_RAD},
  {0.4,-0.25, -0.1, 0,0,0*DEG_TO_RAD},
}
armfsm.firesuppress.larm={  -- For Left arm
	{0.45, 0.3, 0, 0,20*DEG_TO_RAD,-30*DEG_TO_RAD},
}






------------------------------------
-- Associate with the table
Config.arm     = arm
Config.armfsm  = armfsm

return Config
