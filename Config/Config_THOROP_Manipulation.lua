local vector = require'vector'
local DEG_TO_RAD = math.pi/180

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
------   Door pull open
---------------------------------------------------------------
armfsm.dooropen={}

armfsm.dooropen.default_model = {
  0.59,-1.15,0,01,  --Hinge pos
  0.86,             --Door width (hinge to knob axle)
  -0.05,            --Knob X offset from door
  0.08,             --Knob Y offset (from knob axle)
}

--[[
--for old hook
armfsm.dooropen.rhand_rpy={-90*DEG_TO_RAD,-5*DEG_TO_RAD,0}
armfsm.dooropen.rhand_rpy_release={-90*DEG_TO_RAD,45*DEG_TO_RAD,0}
armfsm.dooropen.rhand_rpy_forward={-90*DEG_TO_RAD,5*DEG_TO_RAD,10*DEG_TO_RAD}
armfsm.dooropen.rhand_rpy_sidepush={-0*DEG_TO_RAD,0*DEG_TO_RAD,0*DEG_TO_RAD}
--]]

--With top-mounted hook------------------------------------------------------
armfsm.dooropen.rhand_rpy={0*DEG_TO_RAD,-5*DEG_TO_RAD,0}
armfsm.dooropen.rhand_rpy_release={0*DEG_TO_RAD,45*DEG_TO_RAD,0}
armfsm.dooropen.rhand_rpy_forward={0*DEG_TO_RAD,5*DEG_TO_RAD,10*DEG_TO_RAD}
armfsm.dooropen.rhand_rpy_sidepush={90*DEG_TO_RAD,0*DEG_TO_RAD,0*DEG_TO_RAD}
------------------------------------------------------

armfsm.dooropen.handle_clearance = vector.new({0,0,-0.05})

armfsm.dooropen.rollTarget = -45*DEG_TO_RAD
armfsm.dooropen.yawTargetInitial = 8*DEG_TO_RAD
armfsm.dooropen.yawTarget = 30*DEG_TO_RAD

armfsm.dooropen.velDoorRoll = 10*DEG_TO_RAD * speed_factor
armfsm.dooropen.velDoorYaw = 2*DEG_TO_RAD * speed_factor
armfsm.dooropen.velWaistYaw = 3*DEG_TO_RAD * speed_factor

--hook down instead of up
armfsm.dooropen.handle_clearance = vector.new({0,0,0.05})
armfsm.dooropen.rollTarget = 45*DEG_TO_RAD

-------------------------------------------------------------
--More angle for higher knob
armfsm.dooropen.rhand_rpy={0*DEG_TO_RAD,-30*DEG_TO_RAD,0}
armfsm.dooropen.default_model = {
  0.52,-1.15,0.09,  --Hinge pos
  0.86,             --Door width (hinge to knob axle)
  -0.05,            --Knob X offset from door
  0.08,             --Knob Y offset (from knob axle)
}
armfsm.dooropen.yawTarget = 25*DEG_TO_RAD
---------------------------------------------------------------



--for long arm
--
armfsm.dooropen.default_model = {
  0.58,-1.20,0.09,  --Hinge pos
  0.86,             --Door width (hinge to knob axle)
  -0.05,            --Knob X offset from door
  0.08,             --Knob Y offset (from knob axle)
}
armfsm.dooropen.yawTarget = 45*DEG_TO_RAD
--









---------------------------------------------------------------
------   Door push open
---------------------------------------------------------------
armfsm.doorpush={}

---------------------------------------------------------------
------   Circular valve turning with single hand w/chopstick
---------------------------------------------------------------
armfsm.valveonearm = {}
armfsm.valveonearm.arminit={{0.35,0.30,-0.15, 0,0,0}}

armfsm.valveonearm.velTurnAngle = 6*DEG_TO_RAD * speed_factor
armfsm.valveonearm.velInsert = 0.01 * speed_factor
armfsm.valveonearm.clearance = -0.08

---------------------------------------------------------------
------   Bar valve turning with single hand w/ chopstick
---------------------------------------------------------------
armfsm.valvebar = {}
armfsm.valvebar.default_model= {0.55,0.20,0.07,   0.05, 0, 70*DEG_TO_RAD }
--Axel XYZ, radius, valve angle, hand fit angle

armfsm.valvebar.arminit={{0.35,0.30,-0.15, 0,0,0}}
armfsm.valvebar.handtightangle0 = 45*DEG_TO_RAD
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

------------------------------------
-- Associate with the table
Config.walk    = walk
Config.arm     = arm
Config.armfsm  = armfsm
Config.stance  = stance
Config.zmpstep = zmpstep

return Config
