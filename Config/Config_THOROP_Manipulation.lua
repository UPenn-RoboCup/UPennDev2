local vector = require'vector'
local DEG_TO_RAD = math.pi/180

local Config = {}

Config.IS_LONGARM =true
--Config.IS_LONGARM =false

------------------------------------
-- For the arm FSM
local arm = {}


--Gripper end position offsets (Y is inside)
arm.handoffset={}

--0.130 + 0.60+0.50
arm.handoffset.gripper = {0.241,0,0} --Default gripper
--0.130+0.139+0.80-0.10
arm.handoffset.outerhook = {0.339,0,0.040} --Single hook (for door)
--0.130 + 0.140+0.80-0.10
arm.handoffset.chopstick = {0.340,0,0} --Two rod (for valve)




--Arm planner variables
arm.plan={}
arm.plan.max_margin = math.pi/6
arm.plan.dt_step0 = 0.1
arm.plan.dt_step = 0.2
arm.plan.search_step = 1

-- Arm speed limits
arm.shoulder_yaw_limit = 10*math.pi/180
arm.torso_comp_limit = vector.new({0.02,0.01})

-- Linear movement speed limits
arm.vel_linear_limit = vector.new({0.02,0.02,0.02, 15*DEG_TO_RAD,15*DEG_TO_RAD,15*DEG_TO_RAD})

-- Angular velocity limit at servo
arm.vel_angular_limit = vector.new({10,10,10,15,30,30,30})*DEG_TO_RAD



--testing for speeding up 
arm.overspeed_factor = 3
--arm.overspeed_factor = 1

arm.vel_linear_limit = vector.new({0.06,0.06,0.06, 30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD})
arm.vel_angular_limit = vector.new({20,20,20,20,30,30,30})*DEG_TO_RAD
arm.shoulder_yaw_limit = 20*DEG_TO_RAD

local speed_factor = 1

------------------------------------------------------
--OLD LIMITS FOR WEBOTS
--arm.overspeed_factor = 1
--arm.vel_linear_limit = vector.new({0.06,0.06,0.06, 30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD})
--arm.vel_angular_limit = vector.new({30,30,30,30,90,30,90})*DEG_TO_RAD
arm.shoulder_yaw_limit = 30*DEG_TO_RAD
arm.torso_comp_limit = vector.new({0.06,0.03})
------------------------------------------------------


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
--armfsm.toolgrip.lhand_rpy = {0,0*DEG_TO_RAD, -45*DEG_TO_RAD}
armfsm.toolgrip.lhand_rpy = {0,0*DEG_TO_RAD, 0*DEG_TO_RAD}
armfsm.toolgrip.rhand_rpy = {0,0*DEG_TO_RAD, 45*DEG_TO_RAD}

--xyz, yaw
armfsm.toolgrip.default_model = {
  0.50,-0.06,0.00,  0*DEG_TO_RAD}

armfsm.toolgrip.arminit={
--0.13 -0.13 -0.20
  {0.33,-0.10,-0.20, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.38,-0.10,0.07,  0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
}

armfsm.toolgrip.armhold={0.25,0.0,-0.20}
armfsm.toolgrip.tool_clearance={-0.05,0,0}
armfsm.toolgrip.tool_liftup = {0,0,0.05}
armfsm.toolgrip.tool_clearance_x = 0.38

if Config.IS_LONGARM then --for long arm
  armfsm.toolgrip.arminit={
    {0.33,-0.10,-0.20, 0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
    {0.38,-0.10,0.07,  0,0*DEG_TO_RAD, 45*DEG_TO_RAD},
  }
  armfsm.toolgrip.default_model = {
    0.55,-0.06,0.07,  0*DEG_TO_RAD}

  --Higher
  armfsm.toolgrip.default_model = {
    0.55,-0.06,0.20,  0*DEG_TO_RAD}    
end

---------------------------------------------------------------
------   Drill cut
---------------------------------------------------------------
armfsm.toolchop = {}
armfsm.toolchop.arminit={
  {0.25,-0.20,-0.05},  
  {0.30,-0.0,-0.10},
  {0.35,-0.0,0.0},
}

--For short arm
armfsm.toolchop.curpos={  
  {0.40,0.10,-0.03,0},
  {0.40,-0.20,-0.03,0},
  {0.40,-0.20,0.27,0}
}
--How much torso should follow arm?
armfsm.toolchop.torsoMovementMag = 0.5

armfsm.toolchop.drill_clearance = {-0.05,0,0}


armfsm.toolchop.drill_offset = {0,0,0.10}


if Config.IS_LONGARM then --for long arm
--  armfsm.toolgrip.armhold={0.25,0.0,-0.20}
  armfsm.toolchop.arminit={
    {0.35,-0.0,-0.20},  
    {0.35,-0.0,-0.20},
    {0.40,-0.20,-0.13}, --center pos    
  }
--SJ: we can really barely cut this
  armfsm.toolchop.curpos={
    {0.50,0.16,0.0,0},
    {0.50,-0.36,0.0,0},
    {0.50,-0.36,0.245,0}
  }
  armfsm.toolchop.torsoMovementMag = 0.5
end
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


armfsm.hosegrip.arminit={
--{0.13 0.13 -0.20},
  {0.30,0.13,-0.10},
}

armfsm.hosegrip.armhold = {
  {0.33,0.10,-0.15},
  {0.28,0.35,-0.20},
}
armfsm.hosegrip.clearance={-0.06,0,0}
armfsm.hosegrip.bottompull={0,0,-0.06}

--xyz yaw
armfsm.hosegrip.default_model = {
  0.45,0.09, -0.10, 0}


if Config.IS_LONGARM then
  armfsm.hosegrip.default_model = {
    0.50,0.09, -0.10, 0}
end

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

armfsm.hoseattach.lhand_offset={-0.14,0,0}
armfsm.hoseattach.rhand_offset={-0.08,0,0}

armfsm.hoseattach.larminit={
  {0.40,0.35,-0.20, unpack(armfsm.hoseattach.lhand_rpy0)},
  {0.20,0.05,0.0, unpack(armfsm.hoseattach.lhand_rpy)},
}
armfsm.hoseattach.rarminit={
  {0.40,-0.35,-0.20, unpack(armfsm.hoseattach.rhand_rpy0)},
  {0.28,-0.05,0.0, unpack(armfsm.hoseattach.rhand_rpy)},
}


--xyz yaw pitch
armfsm.hoseattach.default_model = {
  0.40,0.0, 0.10,   0,0}

armfsm.hoseattach.velTurnAngle = 6*DEG_TO_RAD
armfsm.hoseattach.velMove = 0.01

armfsm.hoseattach.angle1 = -30*DEG_TO_RAD
armfsm.hoseattach.angle2 = 30*DEG_TO_RAD

armfsm.hoseattach.clearance={-0.03,0,0}
armfsm.hoseattach.rarm_clearance = 0.05



if Config.IS_LONGARM then
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
  armfsm.hoseattach.default_model = {
    0.45,0.0, 0.10,   0,0}
end







---------------------------------------------------------------
------   Door pull open
---------------------------------------------------------------
armfsm.dooropen={}

armfsm.dooropen.default_model = {
--  0.50,-1.15,0.09,  --Hinge pos
  0.60,-1.18,0.09,  --Hinge pos
  0.84,             --Door width (hinge to knob axle), 33inch for most doors
  -0.05,            --Knob X offset from door
  -0.08,             --Knob Y offset (from knob axle)
}

--With top-mounted hook------------------------------------------------------
armfsm.dooropen.lhand_rpy={0*DEG_TO_RAD,-30*DEG_TO_RAD,0}
armfsm.dooropen.rhand_rpy={0*DEG_TO_RAD,-30*DEG_TO_RAD,0}

armfsm.dooropen.lhand_rpy={0*DEG_TO_RAD,-30*DEG_TO_RAD,-10*DEG_TO_RAD}
armfsm.dooropen.rhand_rpy={0*DEG_TO_RAD,-30*DEG_TO_RAD,-20*DEG_TO_RAD}

armfsm.dooropen.rhand_release={
  {0,0,0,0*DEG_TO_RAD,-30*DEG_TO_RAD,-60*DEG_TO_RAD},
  {0,0,0,0*DEG_TO_RAD,-30*DEG_TO_RAD,-60*DEG_TO_RAD},
  {0,0,0,0*DEG_TO_RAD,85*DEG_TO_RAD,-60*DEG_TO_RAD},
  {0,0,0,0*DEG_TO_RAD,85*DEG_TO_RAD,0*DEG_TO_RAD},
}

armfsm.dooropen.rhand_forward={
  {0,0,0,0*DEG_TO_RAD,85*DEG_TO_RAD, 20*DEG_TO_RAD},
  {0,0,0,0*DEG_TO_RAD,0*DEG_TO_RAD,20*DEG_TO_RAD}
}

armfsm.dooropen.rhand_sidepush={
  {0,0,0,0*DEG_TO_RAD,0*DEG_TO_RAD,-80*DEG_TO_RAD},
  {0,0,0,0*DEG_TO_RAD,85*DEG_TO_RAD,-80*DEG_TO_RAD},
  {0,0,0,0*DEG_TO_RAD,85*DEG_TO_RAD,0*DEG_TO_RAD}
}


------------------------------------------------------
--Hook down
--[[
armfsm.dooropen.handle_clearance0 = vector.new({-0.05,0,0.05})
armfsm.dooropen.handle_clearance = vector.new({0,0,0.05})
armfsm.dooropen.handle_clearance2 = vector.new({0,0,0.05})
armfsm.dooropen.rollTarget = 45*DEG_TO_RAD
--]]


--Hook up
armfsm.dooropen.handle_clearance0 = vector.new({-0.05,0,-0.05})
armfsm.dooropen.handle_clearance1 = vector.new({0,0,-0.05})
armfsm.dooropen.handle_clearance2 = vector.new({0,0,-0.10})
armfsm.dooropen.rollTarget = -45*DEG_TO_RAD

armfsm.dooropen.yawTargetInitial = 8*DEG_TO_RAD
armfsm.dooropen.yawTarget1 = 25*DEG_TO_RAD
armfsm.dooropen.yawTarget2 = 24*DEG_TO_RAD

armfsm.dooropen.velDoorRoll = 10*DEG_TO_RAD * speed_factor
armfsm.dooropen.velDoorYaw = 2*DEG_TO_RAD * speed_factor
armfsm.dooropen.velWaistYaw = 3*DEG_TO_RAD * speed_factor

if Config.IS_LONGARM then
  armfsm.dooropen.default_model = {
    0.62,-1.16,0.09,  --Hinge pos
    0.84,             --Door width (hinge to knob axle)
    -0.05,            --Knob X offset from door
    -0.08,             --Knob Y offset (from knob axle)
  }
end






---------------------------------------------------------------
------   Door push open
---------------------------------------------------------------
armfsm.dooropenleft={}



armfsm.dooropenleft.default_model = {
  0.51,-0.53,0.09,  --Hinge pos
  0.84,             --Door width (hinge to knob axle)
  -0.05,            --Knob X offset from door
  -0.08,             --Knob Y offset (from knob axle)
}

armfsm.dooropenleft.rollTarget = -45*DEG_TO_RAD
armfsm.dooropenleft.yawTargetInitial = -8*DEG_TO_RAD

if Config.IS_LONGARM then
  armfsm.dooropenleft.default_model = {
    0.52,-0.50,0.09,  --Hinge pos
    0.84,             --Door width (hinge to knob axle)
    -0.05,            --Knob X offset from door
    -0.08,             --Knob Y offset (from knob axle)
  }
end

armfsm.dooropenleft.lhand_rpy=
  {0,0,0,0*DEG_TO_RAD,-0*DEG_TO_RAD,-10*DEG_TO_RAD}

armfsm.dooropenleft.rhand_push={
  {0,0,0,0*DEG_TO_RAD,-0*DEG_TO_RAD, 45*DEG_TO_RAD}, 
--0.21 -0.06 -0.16
  {0.31, -0.0, -0.10,0*DEG_TO_RAD,-0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.65, -0.05, -0.0,0*DEG_TO_RAD,-0*DEG_TO_RAD, 45*DEG_TO_RAD},
  {0.65, -0.05, -0.0,0*DEG_TO_RAD,-0*DEG_TO_RAD, -30*DEG_TO_RAD},
}






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

if Config.IS_LONGARM then
  armfsm.dooredge.hinge_offset_z = -0.29 
  armfsm.dooredge.door_yaw1 = 15*DEG_TO_RAD
  armfsm.dooredge.door_yaw2 = 55*DEG_TO_RAD
end



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
  {0.60,0.30,0.07, 0, -60*DEG_TO_RAD, 60*DEG_TO_RAD }

--height should be between 0.81(-0.12) to 1.22 (+0.29)

armfsm.valveonearm.default_model_large= 
  {0.69,0.27,0.07, 0.13, -60*DEG_TO_RAD, 60*DEG_TO_RAD }

--boundary between low/med/high
armfsm.valveonearm.heights={0.03,0.09}

armfsm.valveonearm.arminit={
  {0.55,0.30,-0.12, 0,0,0}, --low
  {0.55,0.40, 0.07, 0,0,0}, --med
  {0.55,0.30, 0.29, 0,0,0}, --high
}

if Config.IS_LONGARM then --for long arm
  armfsm.valveonearm.default_model_small= 
    {0.70,0.30,0.07, 0, -60*DEG_TO_RAD, 60*DEG_TO_RAD }
--[[
  armfsm.valveonearm.default_model_small= 
    {0.70,0.30,-0.12, 0, -60*DEG_TO_RAD, 60*DEG_TO_RAD }
  armfsm.valveonearm.default_model_small= 
    {0.70,0.30,0.29, 0, -60*DEG_TO_RAD, 60*DEG_TO_RAD }
--]]
  armfsm.valveonearm.arminit={
    {0.62,0.30,-0.12, 0,0,0}, --low
    {0.62,0.40, 0.07, 0,0,0}, --med
    {0.62,0.30, 0.29, 0,0,0}, --high
  }
end

armfsm.valveonearm.clearance = -0.08
armfsm.valveonearm.velTurnAngle = 6*DEG_TO_RAD * speed_factor
armfsm.valveonearm.velInsert = 0.01 * speed_factor
---------------------------------------------------------------
------   Bar valve turning with single hand w/ chopstick
---------------------------------------------------------------
armfsm.valvebar = {}
--Axel XYZ, radius, current valve angle, final valve angle, hand fit angle, 
--valve angle zero: straight UP
armfsm.valvebar.default_model= {0.65,0.20,0.07,   
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
---- Arm folding
--------------------------------------------------------------------------

armfsm.rocky={}



armfsm.rocky.lhand_rpy0 = {90*DEG_TO_RAD,-25*DEG_TO_RAD,0}
armfsm.rocky.rhand_rpy0 = {-90*DEG_TO_RAD,-25*DEG_TO_RAD,0}
armfsm.rocky.lhand_rpy1 = {90*DEG_TO_RAD,-65*DEG_TO_RAD,0}
armfsm.rocky.rhand_rpy1 = {-90*DEG_TO_RAD,-65*DEG_TO_RAD,0}
armfsm.rocky.lhand_rpy2 = {90*DEG_TO_RAD,-75*DEG_TO_RAD,0}
armfsm.rocky.rhand_rpy2 = {-90*DEG_TO_RAD,-75*DEG_TO_RAD,0}

armfsm.rocky.lhand_rpy3 = {90*DEG_TO_RAD,40*DEG_TO_RAD,0}
armfsm.rocky.rhand_rpy3 = {-90*DEG_TO_RAD,40*DEG_TO_RAD,0}



armfsm.rocky.larminit={
  {0.35,0.242,0,unpack(armfsm.rocky.lhand_rpy0)},
  {0.35,0.22,0,unpack(armfsm.rocky.lhand_rpy1)}
}
armfsm.rocky.rarminit={
  {0.35,-0.242,0,unpack(armfsm.rocky.rhand_rpy0)},
  {0.35,-0.22,0,unpack(armfsm.rocky.rhand_rpy1)}
}


--for longarm
armfsm.rocky.larminit={
  {0.35,0.30,0,unpack(armfsm.rocky.lhand_rpy0)},
  {0.35,0.30,0,unpack(armfsm.rocky.lhand_rpy1)},
  {0.30,0.40,0.21,unpack(armfsm.rocky.lhand_rpy2)},
  {0.40,0.45,0.61,unpack(armfsm.rocky.lhand_rpy2)},
  {0.40,0.45,0.61,unpack(armfsm.rocky.lhand_rpy3)}
}
armfsm.rocky.larminit={
  {0.35,0.30,0,unpack(armfsm.rocky.lhand_rpy0)},
  {0.35,0.30,0,unpack(armfsm.rocky.lhand_rpy1)},
  {0.30,0.40,0.21,unpack(armfsm.rocky.lhand_rpy2)},
  {0.40,0.45,0.61,unpack(armfsm.rocky.lhand_rpy2)},
  {0.40,0.45,0.61,unpack(armfsm.rocky.lhand_rpy3)}
}
armfsm.rocky.rarminit={
  {0.35,-0.30,0,unpack(armfsm.rocky.rhand_rpy0)},
  {0.35,-0.30,0,unpack(armfsm.rocky.rhand_rpy1)},
  {0.30,-0.40,0.21,unpack(armfsm.rocky.rhand_rpy2)},
  {0.40,-0.45,0.61,unpack(armfsm.rocky.rhand_rpy2)},
  {0.40,-0.45,0.61,unpack(armfsm.rocky.rhand_rpy3)}
}


--------------------------------------------------------------------------
---- FOR LONG ARM (25cm)
--------------------------------------------------------------------------






------------------------------------
-- Associate with the table
Config.walk    = walk
Config.arm     = arm
Config.armfsm  = armfsm
Config.stance  = stance
Config.zmpstep = zmpstep

return Config
