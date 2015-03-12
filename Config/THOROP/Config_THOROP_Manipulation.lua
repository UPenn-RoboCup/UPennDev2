assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'
local carray = require'carray'

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



--New 3 finger gripper
arm.handoffset.gripper3 = {0.28,-0.05,0} 



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

arm.plan.dt_step_min = 0.02
arm.plan.dt_step_min = 0.2 --no speedup

--arm.plan.dt_step_min = 0.1 --2x speedup


--arm.plan.dt_step0 = 0.05
--arm.plan.dt_step = 0.1



arm.plan.search_step = 0.25

-- Arm speed limits
arm.torso_comp_limit = vector.new({0.02,0.01})

--testing for speeding up
arm.overspeed_factor = 3
--arm.overspeed_factor = 1

arm.vel_angular_limit = vector.new({20,20,20,20,30,30,30})*DEG_TO_RAD
arm.vel_angular_limit_init = vector.new({20,20,20,20,30,30,30})*DEG_TO_RAD
arm.vel_linear_limit = vector.new({0.02,0.02,0.02, 30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD})
arm.vel_waist_limit = vector.new({3,3})*DEG_TO_RAD
arm.shoulder_yaw_limit = 20*DEG_TO_RAD

--if IS_WEBOTS then 
if false then    
  speedup_factor = 3
  arm.shoulder_yaw_limit = arm.shoulder_yaw_limit*speedup_factor
  arm.vel_linear_limit = arm.vel_linear_limit*speedup_factor
  arm.vel_angular_limit = arm.vel_angular_limit*speedup_factor
  arm.vel_angular_limit_init = arm.vel_angular_limit_init*3 
end





--Servo limit
-- 30rpm 30rpm 30rpm 30rpm 20rpm 20rpm 20rpm
-- = 180 180 180 180 120 120 120 degree/sec

local speed_factor = 1

------------------------------------------------------
--OLD LIMITS FOR WEBOTS
--arm.overspeed_factor = 1arm_planner:set_shoulder_yaw_target(qLArm0[3], nil) --Lock left hand
--arm.vel_linear_limit = vector.new({0.06,0.06,0.06, 30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD})
--arm.vel_angular_limit = vector.new({30,30,30,30,90,30,90})*DEG_TO_RAD
arm.shoulder_yaw_limit = 30*DEG_TO_RAD
arm.torso_comp_limit = vector.new({0.06,0.03})
------------------------------------------------------



--Pose 1 left and right arm poses 
--SJ: now arm ready poses are hand-invariant (to handle asymmetric hands and stuff)
arm.trLArm0 = {0.0, 0.25,-0.25,0,0,0}
arm.trRArm0 = {0.0, -0.25,-0.25,0,0,0}

--arm.trLArm1 = {0.0, 0.25,-0.25,0,0,-45*DEG_TO_RAD}
--arm.trRArm1 = {0.0, -0.25,-0.25,0,0,45*DEG_TO_RAD}

--arm.ShoulderYaw0 = {5*DEG_TO_RAD,-5*DEG_TO_RAD}
arm.ShoulderYaw0=vector.new({0.1,-0.1})*DEG_TO_RAD

--Arm State specific infos
armfsm = {}



---------------------------------------------------------------
------   Drill pickup
---------------------------------------------------------------
armfsm.toolgrip = {}
armfsm.toolgrip.lhand_rpy = {0,0,-45*DEG_TO_RAD}
armfsm.toolgrip.rhand_rpy = {0,0,45*DEG_TO_RAD}

--Conservative initial model (away from target)
armfsm.toolgrip.default_model = {0.42,-0.06,0.20,  0*DEG_TO_RAD}

armfsm.toolgrip.arminit={
  {'move',nil,{0.29,-0.40,-0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
  {'move',nil,{0.46,-0.40,0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
}

armfsm.toolgrip.armuninit={
  {'move',nil,{0.46,-0.40,0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
  {'move',nil,{0.29,-0.40,-0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
  {'move',nil,arm.trRArm1},
  {'move',nil,arm.trRArm0},
}

armfsm.toolgrip.armpull={
  {'wrist',nil,{0.42,-0.40,0.20,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
  {'move',nil,{0.42,-0.40,0.20,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
  {'move',nil,{0.42,-0.40,0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
  {'move',nil,{0.29,-0.40,-0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
  {'move',nil,{0.25,0.0,-0.20,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
}

--0.23 offset = (0.16,0.16) offset
armfsm.toolgrip.arminit={
  {'move0',nil,{0.13,-0.56,-0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
  {'move0',nil,{0.30,-0.56,0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
}

armfsm.toolgrip.armuninit={
  {'move0',nil,{0.30,-0.56,0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
  {'move0',nil,{0.13,-0.56,-0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
--  {'move0',nil,arm.trRArm1},
  {'move0',nil,arm.trRArm0},
}




armfsm.teleop = {}

--[[
armfsm.teleop.arminit={
  {'move0',nil,{0.10,-0.20,-0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
}
armfsm.teleop.armuninit={
  {'move0',nil,{0.0,-0.25,-0.25,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},  
}
--]]

--FOR DRC TESTBED
--Table height: 1.04 from grpund, 0.11 from waist center
--Drill handle height: 1.15 from ground, 0.22 from waist center
--Valve center: 1.09 from ground, 0.16 from waist center
--Hose center: 1.25 from groundm 0.32 from waist center


--Drill?
armfsm.teleop.arminit={  
  {'move0',nil,{0.20,-0.25,-0.15,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
  {'move',nil,{0.40,-0.05, 0.22,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
}
armfsm.teleop.armuninit={
  {'move',nil,{0.40,-0.05, 0.0,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},
  {'move0',nil,{0.0,-0.25,-0.25,0,0*DEG_TO_RAD, 45*DEG_TO_RAD}},  
}


--Straight hand front (door?)
armfsm.teleop.rhand_rpy0={-90*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.teleop.arminit={  
  {'move0',nil,{0.0,-0.25,-0.15,0,0*DEG_TO_RAD, 0*DEG_TO_RAD}},
  {'move0',nil,{0.0,-0.25,-0.15,-90*DEG_TO_RAD,0*DEG_TO_RAD, 0*DEG_TO_RAD}},
}
armfsm.teleop.armuninit={  
  {'move0',nil,{0.0,-0.25,-0.15,0,0*DEG_TO_RAD, 0*DEG_TO_RAD}},
  {'move0',nil,{0.0,-0.25,-0.25,0,0*DEG_TO_RAD, 0*DEG_TO_RAD}},  
}

--[[
--Hose reinsert?
armfsm.teleop.rhand_rpy0={0*DEG_TO_RAD,-40*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.teleop.arminit={  
  {'move',nil,{0.35,-0.30,-0.05,0,0*DEG_TO_RAD, 0*DEG_TO_RAD}},
  {'move',nil,{0.35,-0.30,0.05,0,-45*DEG_TO_RAD, 0*DEG_TO_RAD}},  
  {'move',nil,{0.35,-0.30,0.15,0,-60*DEG_TO_RAD, 0*DEG_TO_RAD}},  
  {'move',nil,{0.30,-0.30,0.30,0,-85*DEG_TO_RAD, 0*DEG_TO_RAD}},  
  {'move',nil,{0.30,-0.30,0.30,0,-89*DEG_TO_RAD, 0*DEG_TO_RAD}},

  {'move',nil,{0.30,-0.30,0.30,45*DEG_TO_RAD,-89*DEG_TO_RAD, 0}},  
}
armfsm.teleop.armuninit={  
  {'move0',nil,{0.0,-0.25,-0.15,0,0*DEG_TO_RAD, 0*DEG_TO_RAD}},
  {'move0',nil,{0.0,-0.25,-0.25,0,0*DEG_TO_RAD, 0*DEG_TO_RAD}},  
}
--]]


--Hose insert
--Init tr: 0.24 -0.13 0.14 (90.0 -45.0 95.1)
--0.40 0.15 0.30 (90.0 -45.0 95.1)
--0.40 -0.17 0.30 









--finer grain search
--arm.plan.dt_step0 = 0.05
--arm.plan.dt_step = 0.1




------------------------------------------------------------------------------------
-- High arm position testing


--TRL: 0.14 0.23 0.35 (-180.0 -80.0 180.0)

--arm.ShoulderYaw0=vector.new({0,0})*DEG_TO_RAD

--Arm State specific infos
--armfsm = {}


--]]


--[[
--Load arm IK lookup table
local fname = {Config.PLATFORM_NAME,'/iklookup'}  
local c = require(table.concat(fname))
if c.iklookup then 
  arm.iklookup={}
  arm.iklookup.x = c.iklookup.x
  arm.iklookup.y = c.iklookup.y
  arm.iklookup.z = c.iklookup.z

  print("ik table size:",#c.iklookup.dat)
  arm.iklookup.dat=carray.int(#c.iklookup.dat)
  for i=1,#c.iklookup.dat do
    arm.iklookup.dat[i]=c.iklookup.dat[i]
  end
end
--]]


------------------------------------
-- Associate with the table
Config.arm     = arm
Config.armfsm  = armfsm



return Config
