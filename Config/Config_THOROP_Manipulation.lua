local vector = require'vector'
local DEG_TO_RAD = math.pi/180

local IS_LONGARM = true
local IS_LONGARM = false


local Config = {}

------------------------------------
-- For the arm FSM
local arm = {}


--Gripper end position offsets (Y is inside)
arm.handoffset={}
arm.handoffset.gripper = {0.245,0.035,0} --Default gripper

--For older hook
--arm.handoffset.outerhook = {0.285,-0.065,0} --Single hook (for door)

--For new hook
arm.handoffset.outerhook = {0.285,0,0.065} --Single hook (for door)

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



-- Arm speed limits

arm.wrist_turn_limit = vector.new({0,0,0,15*DEG_TO_RAD,15*DEG_TO_RAD,15*DEG_TO_RAD})
arm.shoulder_yaw_limit = 10*math.pi/180
arm.torso_comp_limit = vector.new({0.02,0.01})

-- Use this for planned arm movement
arm.joint_vel_limit_plan = vector.new({10,10,10,10,30,10,30}) *DEG_TO_RAD

-- vel limit at the servo level
arm.slow_limit = vector.new({10,10,10,15,30,30,30})*DEG_TO_RAD

-- Linear movement speed limits
arm.linear_slow_limit = vector.new({0.02,0.02,0.02, 15*DEG_TO_RAD,15*DEG_TO_RAD,15*DEG_TO_RAD})


local speed_factor = 1
if IS_WEBOTS then
  speed_factor = 3  
end




arm.wrist_turn_limit = arm.wrist_turn_limit * speed_factor
arm.shoulder_yaw_limit = arm.shoulder_yaw_limit * speed_factor
arm.torso_comp_limit = arm.torso_comp_limit * speed_factor
arm.slow_limit = arm.slow_limit * speed_factor
arm.linear_slow_limit = arm.linear_slow_limit * speed_factor
arm.joint_vel_limit_plan = arm.joint_vel_limit_plan * speed_factor



--Pose 1 wrist position
arm.pLWristTarget0 = {-.0,.30,-.20,0,0,0}
arm.pRWristTarget0 = {-.0,-.30,-.20,0,0,0}

--For old hook
--[[
arm.lrpy0 = vector.new({0,0,0,90,30,0})*DEG_TO_RAD
arm.rrpy0 = vector.new({0,0,0,-90,30,0})*DEG_TO_RAD
--]]

--For new hook
arm.lrpy0 = vector.new({0,0,0,0,30,0})*DEG_TO_RAD
arm.rrpy0 = vector.new({0,0,0,-0,30,0})*DEG_TO_RAD

arm.lShoulderYaw0 = -5*DEG_TO_RAD
arm.rShoulderYaw0 = 5*DEG_TO_RAD


arm.qLArmPose1 = vector.new({118.96025904076,9.0742631178663,-5,-81.120944928286,81,14.999999999986, 9})*DEG_TO_RAD
arm.qRArmPose1 = vector.new({118.96025904076,-9.0742631178663,5,-81.120944928286,-81,-14.999999999986, 9})*DEG_TO_RAD


--Arm State specific infos
armfsm = {}

---------------------------------------------------------------
------   Drill pickup 
---------------------------------------------------------------
armfsm.toolgrip = {}
armfsm.toolgrip.lhand_rpy = {0,0*DEG_TO_RAD, -45*DEG_TO_RAD}
armfsm.toolgrip.rhand_rpy = {0,0*DEG_TO_RAD, 45*DEG_TO_RAD}

--xyz, yaw
armfsm.toolgrip.default_model = {
  0.49,-0.02,0.00,  0*DEG_TO_RAD}

armfsm.toolgrip.arminit={
  {0.25,-0.10,-0.05},  
  {0.30,-0.10,-0.10},
  {0.35,-0.10,0.05},
}
armfsm.toolgrip.armhold={ {0.20,0.0,-0.10}}
armfsm.toolgrip.tool_clearance={-0.08,0,0}
armfsm.toolgrip.tool_liftup = {0,0,0.05}
armfsm.toolgrip.tool_liftuppull = {-0.20,0,0.05}

---------------------------------------------------------------
------   Drill cut
---------------------------------------------------------------
armfsm.toolchop = {}
armfsm.toolchop.arminit={
  {0.25,-0.20,-0.05},  
  {0.30,-0.0,-0.10},
  {0.35,-0.0,0.0},
}
armfsm.toolchop.drill_clearance = {-0.05,0,0}



---------------------------------------------------------------
------   Hose pickup
---------------------------------------------------------------
armfsm.hosegrip = {}
armfsm.hosegrip.lhand_rpy = {-90*DEG_TO_RAD,45*DEG_TO_RAD,  0*DEG_TO_RAD}
armfsm.hosegrip.rhand_rpy = {90*DEG_TO_RAD,45*DEG_TO_RAD, 0*DEG_TO_RAD}





armfsm.hosegrip.lhand_rpy = {-90*DEG_TO_RAD,0*DEG_TO_RAD,  0*DEG_TO_RAD}



armfsm.hosegrip.arminit={
--  {0.21, 0,30, -0.16, unpack(armfsm.hosegrip.lhand_rpy)},
  {0.21,0.40,-0.06, unpack(armfsm.hosegrip.lhand_rpy)},
  {0.30,0.55,0.10, unpack(armfsm.hosegrip.lhand_rpy)},
  {0.35,0.50,0.16, unpack(armfsm.hosegrip.lhand_rpy)},  
  {0.40,0.45,0.16, unpack(armfsm.hosegrip.lhand_rpy)},    
--  {0.45,0.40,0.25, unpack(armfsm.hosegrip.lhand_rpy)},    

  {0.50,0.40,0.30, unpack(armfsm.hosegrip.lhand_rpy)},    
  

--{0.45,0.50,0.30, unpack(armfsm.hosegrip.lhand_rpy)},    


--  {0.30,0.50,0.20, unpack(armfsm.hosegrip.lhand_rpy)},

}



--xyz yaw
armfsm.hosegrip.default_model = {
  0.50,0.30, 0.07, 0}

armfsm.hosegrip.clearance={0,0,0.08}


--[[
armfsm.hosegrip.armflip={
  vector.new({160,0,0,-140,90,40,0})*DEG_TO_RAD,   
  vector.new({90,0,0,-140,90,40,0})*DEG_TO_RAD, 
  vector.new({90,150,0,-140,90,40,0})*DEG_TO_RAD, 
  vector.new({90,150,0,-140,90,-10,0})*DEG_TO_RAD,
}

armfsm.hosegrip.arminit={
--  {0.42,0.30,0.08, unpack(armfsm.hosegrip.lhand_rpy)},
  {0.42,0.30,0.20, unpack(armfsm.hosegrip.lhand_rpy)},
  {0.52,0.30,0.20, unpack(armfsm.hosegrip.lhand_rpy)},
}

armfsm.hosegrip.armuninit={
  {0.41,0.23,0.08, -132.9*DEG_TO_RAD, 43.8*DEG_TO_RAD, -26.5*DEG_TO_RAD},
}
--91 150 0 -138 -84 10 174 
--SJ: we need to flip the wrist angle first :[
armfsm.hosegrip.armunflip={
  vector.new({90,150,0,-140,-90,10,180})*DEG_TO_RAD,
  vector.new({90,150,0,-140,-89,0,181})*DEG_TO_RAD,
  vector.new({90,150,0,-140, 90,0,0})*DEG_TO_RAD,
  vector.new({90,150,0,-140,90,40,0})*DEG_TO_RAD, 
  vector.new({90,0,0,-140,90,40,0})*DEG_TO_RAD, 
  vector.new({160,0,0,-140,90,40,0})*DEG_TO_RAD,   
  vector.new({124,12,0,-80,85,14,-98})*DEG_TO_RAD,   
}
--]]













---------------------------------------------------------------
------   Door pull open
---------------------------------------------------------------
armfsm.dooropen={}

armfsm.dooropen.default_model = {
  0.50,-1.15,0.09,  --Hinge pos
  0.86,             --Door width (hinge to knob axle)
  -0.05,            --Knob X offset from door
  0.11,             --Knob Y offset (from knob axle)
}

--With top-mounted hook------------------------------------------------------
armfsm.dooropen.rhand_rpy={0*DEG_TO_RAD,-30*DEG_TO_RAD,0}
armfsm.dooropen.rhand_rpy_release={0*DEG_TO_RAD,45*DEG_TO_RAD,0}

armfsm.dooropen.rhand_rpy_release={0*DEG_TO_RAD,-30*DEG_TO_RAD,-80*DEG_TO_RAD}
armfsm.dooropen.rhand_rpy_release2={0*DEG_TO_RAD,70*DEG_TO_RAD,-80*DEG_TO_RAD}

armfsm.dooropen.rhand_rpy_release3={0*DEG_TO_RAD,85*DEG_TO_RAD,0*DEG_TO_RAD}

armfsm.dooropen.rhand_rpy_forward={0*DEG_TO_RAD,5*DEG_TO_RAD,10*DEG_TO_RAD}
armfsm.dooropen.rhand_rpy_sidepush={90*DEG_TO_RAD,0*DEG_TO_RAD,0*DEG_TO_RAD}
------------------------------------------------------

armfsm.dooropen.handle_clearance0 = vector.new({-0.05,0,0.05})
armfsm.dooropen.handle_clearance = vector.new({0,0,0.05})
armfsm.dooropen.rollTarget = 45*DEG_TO_RAD
armfsm.dooropen.yawTargetInitial = 8*DEG_TO_RAD
armfsm.dooropen.yawTarget = 30*DEG_TO_RAD

armfsm.dooropen.velDoorRoll = 10*DEG_TO_RAD * speed_factor
armfsm.dooropen.velDoorYaw = 2*DEG_TO_RAD * speed_factor
armfsm.dooropen.velWaistYaw = 3*DEG_TO_RAD * speed_factor



armfsm.dooropen.yawTarget = 25*DEG_TO_RAD






--swing open phase after unlock the door 
  
armfsm.dooredge={}
armfsm.dooredge.hinge_offset_z = -0.26 --how low do we touch?

armfsm.dooredge.hand_offset_x = 0.20
armfsm.dooredge.edge_offset_x = 0.05
armfsm.dooredge.edge_offset_y = 0.05

armfsm.dooredge.rhand_rpy={90*DEG_TO_RAD,45*DEG_TO_RAD,0}
armfsm.dooredge.edge_clearance = vector.new({0,0.05,0})

armfsm.dooredge.door_yaw1 = 15*DEG_TO_RAD --start angle
armfsm.dooredge.door_yaw2 = 50*DEG_TO_RAD --end angle


---------------------------------------------------------------
------   Door push open
---------------------------------------------------------------
armfsm.doorpush={}

armfsm.doorpush.default_model = {
  0.50,-0.50,0.09,  --Hinge pos
  0.86,             --Door width (hinge to knob axle)
  -0.05,            --Knob X offset from door
  0.08,             --Knob Y offset (from knob axle)
}

armfsm.doorpush.roll1 = 0*DEG_TO_RAD
armfsm.doorpush.roll2 = 135*DEG_TO_RAD
armfsm.doorpush.clearance = {-0.08,0,0}

armfsm.doorpush.yawTargetInitial = -8*DEG_TO_RAD
armfsm.doorpush.yawTarget = -10*DEG_TO_RAD




armfsm.doorsupport={}

armfsm.doorsupport.lhand_rpy = {0,0*DEG_TO_RAD, -45*DEG_TO_RAD}
armfsm.doorsupport.lhand_rpy2 = {0,0*DEG_TO_RAD, -90*DEG_TO_RAD}

armfsm.doorsupport.arminit={
  {0.40,0.10,-0.05,0,0,-45*DEG_TO_RAD},
  {0.26,-0.15, 0.15,unpack(armfsm.doorsupport.lhand_rpy2)},
  {0.26,-0.20, 0.15,unpack(armfsm.doorsupport.lhand_rpy2)},
}












---------------------------------------------------------------
------   Circular valve turning with single hand w/chopstick
---------------------------------------------------------------
armfsm.valveonearm = {}

--Axel XYZ, radius, valve angle 1, valve angle 2
armfsm.valveonearm.default_model_small= 
  {0.55,0.30,0.07, 
    0, -60*DEG_TO_RAD, 60*DEG_TO_RAD }

armfsm.valveonearm.default_model_large= 
  {0.55,0.27,0.07,   
    0.13, -60*DEG_TO_RAD, 60*DEG_TO_RAD }

armfsm.valveonearm.arminit={{0.35,0.30,-0.15, 0,0,0}}
armfsm.valveonearm.clearance = -0.08

armfsm.valveonearm.velTurnAngle = 6*DEG_TO_RAD * speed_factor
armfsm.valveonearm.velInsert = 0.01 * speed_factor
---------------------------------------------------------------
------   Bar valve turning with single hand w/ chopstick
---------------------------------------------------------------
armfsm.valvebar = {}
--Axel XYZ, radius, current valve angle, final valve angle, hand fit angle, 
--valve angle zero: straight down
armfsm.valvebar.default_model= {0.55,0.20,0.07,   
  0.05, 0, 90*DEG_TO_RAD, -10*DEG_TO_RAD  }

armfsm.valvebar.arminit={{0.35,0.30,-0.15, 0,0,0}}
armfsm.valvebar.handtightangle0 = -45*DEG_TO_RAD
armfsm.valvebar.clearance = -0.08

armfsm.valvebar.velTurnAngle = 6*DEG_TO_RAD * speed_factor
armfsm.valvebar.velInsert = 0.01 * speed_factor
---------------------------------------------------------------
------   Two-armed large valve turning (with chopstick)
---------------------------------------------------------------
armfsm.valvetwoarm = {0.55,0.20,0.07,   0.05, 0, 70*DEG_TO_RAD }

--Axel XYZ, radius, valve angle, hand fit angle
armfsm.valvetwoarm.velTurnAngle = 3*DEG_TO_RAD * speed_factor
armfsm.valvetwoarm.velInsert = 0.02 * speed_factor





---------------------------------------------------------------
------   Debris pickup
---------------------------------------------------------------
armfsm.debrisgrip = {}
armfsm.debrisgrip.lhand_rpy = {-90*DEG_TO_RAD,45*DEG_TO_RAD,  0*DEG_TO_RAD}
armfsm.debrisgrip.rhand_rpy = {90*DEG_TO_RAD,45*DEG_TO_RAD, 0*DEG_TO_RAD}



















--------------------------------------------------------------------------
---- FOR LONG ARM (25cm)
--------------------------------------------------------------------------

if IS_LONGARM then

armfsm.dooropen.default_model = {
  0.58,-1.20,0.09,  --Hinge pos
  0.86,             --Door width (hinge to knob axle)
  -0.05,            --Knob X offset from door
  0.08,             --Knob Y offset (from knob axle)
}
armfsm.dooropen.yawTarget = 45*DEG_TO_RAD


armfsm.dooredge.hinge_offset_z = -0.29 
armfsm.dooredge.door_yaw1 = 15*DEG_TO_RAD
armfsm.dooredge.door_yaw2 = 55*DEG_TO_RAD


end










------------------------------------
-- Associate with the table
Config.walk    = walk
Config.arm     = arm
Config.armfsm  = armfsm
Config.stance  = stance
Config.zmpstep = zmpstep

return Config
