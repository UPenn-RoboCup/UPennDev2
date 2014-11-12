--Stance state is basically a Walk controller
--Without any torso or feet update
--We share the leg joint generation / balancing code 
--with walk controllers

local state = {}
state._NAME = ...

local Body   = require'Body'
local K      = Body.Kinematics
local vector = require'vector'
local unix   = require'unix'
local util   = require'util'
local moveleg = require'moveleg'
local movearm = require'movearm'

require'mcm'
require'hcm'

local simple_ipc = require'simple_ipc'
local head_ch   = simple_ipc.new_publisher('HeadFSM!')

-- Keep track of important times
local t_entry, t_update, t_last_step

local qLArm0, qRArm0,qLLeg0,qRLeg0,qWaist0
local qLArm1, qRArm1,qLLeg1,qRLeg1,qWaist1



local slow_down_factor = 0.8

local qLArm_initial = Body.get_larm_command_position()
local qRArm_initial = Body.get_rarm_command_position()

local qL_WristTarget_pos = {0.2,0.35,-0.06,0,0,0}-- six component vector;
local qR_WristTarget_pos = {0.2,-0.35,-0.06,0,0,0}-- six component vector;
local qR_WristTarget_pos3 = {0.2,-0.35,-0.25,0,0,0}-- six component vector;

local qL_Target_rpy1 = vector.new({0, 0,  0,  0,  45,  0})*DEG_TO_RAD
local qR_Target_rpy1 = vector.new({0, 0,  0,  -0, 45,  0})*DEG_TO_RAD
local qL_Target_rpy2 = vector.new({0, 0,  0,  60,  -45,  0})*DEG_TO_RAD
local qR_Target_rpy2 = vector.new({0, 0,  0,  -30, 0,  0})*DEG_TO_RAD
local qR_Target_rpy3 = vector.new({0, 0,  0,  -90, 0,  0})*DEG_TO_RAD

local lShoulderYaw = qLArm_initial[3];
local rShoulderYaw = qRArm_initial[3];

local qLWrist = Body.get_inverse_lwrist(qLArm_initial,qL_WristTarget_pos,lShoulderYaw)
local qRWrist = Body.get_inverse_rwrist(qRArm_initial,qR_WristTarget_pos,rShoulderYaw)
local qRWrist3 = Body.get_inverse_rwrist(qRArm_initial,qR_WristTarget_pos3,rShoulderYaw)

local qLArmTarget1 = Body.get_inverse_arm_given_wrist(qLWrist, qL_Target_rpy1)
local qRArmTarget1 = Body.get_inverse_arm_given_wrist(qRWrist, qR_Target_rpy1)
local qLArmTarget2 = Body.get_inverse_arm_given_wrist(qLWrist, qL_Target_rpy2)
local qRArmTarget2 = Body.get_inverse_arm_given_wrist(qRWrist, qR_Target_rpy2)
local qRArmTarget3 = Body.get_inverse_arm_given_wrist(qRWrist3, qR_Target_rpy3)

local trRleg = {0.3,  -0.3, -0.2,0,0,0}
local trLleg = {0.3,   0.3, -0.2,0,0,0}

--local qRlegTarget = Body.get_inverse_lleg(trLleg)
--local qLlegTarget = Body.get_inverse_rleg(trRleg)

unpack( vector.new(qLArmTarget1)*RAD_TO_DEG)
unpack( vector.new(qRArmTarget1)*RAD_TO_DEG)
unpack( vector.new(qRArmTarget2)*RAD_TO_DEG)
unpack( vector.new(qLArmTarget2)*RAD_TO_DEG)
unpack( vector.new(qRArmTarget3)*RAD_TO_DEG)

--print(string.format("QRLegTarget: %.2f %.2f %.2f %.2f", unpack( vector.new(qRlegTarget)*RAD_TO_DEG)))
--print(string.format("QLLegTarget: %.2f %.2f %.2f %.2f", unpack( vector.new(qLlegTarget)*RAD_TO_DEG)))

local keyframe={
--#1
  {  
    qLArm = qLArmTarget1*RAD_TO_DEG,--{140,  0,  5, -98,  42,  16, -62},    
    qRArm = qRArmTarget1*RAD_TO_DEG,--{140, -0,  5, -98, -42, -16, -62},     
    qLLeg = {0.12,    2.30,  -25.00,   10.00,  -30.00,   -0.00},
    qRLeg = {-0.12,   -2.30, -25.00,   10.00,  -30.00,    0.00},
    duration = 2.0*slow_down_factor,
  },
--#2
  {  
    qLArm = qLArmTarget1*RAD_TO_DEG,--{140,  0,  5, -98,  42,  16, -62},    
    qRArm = qRArmTarget1*RAD_TO_DEG,--{140, -0,  5, -98, -42, -16, -62},     
    qLLeg = {15.00,    2.30,  -25.00,   10.00,  -30.00,   -0.00},
    qRLeg = {-15.00,   -2.30,  -25.00,   10.00,  -30.00,    0.00},
    qWaist = {0,-15},
    duration = 3.0*slow_down_factor,
  },
--#3
  {  
    qLArm = qLArmTarget2*RAD_TO_DEG,--{140,  0,  5, -98,  42,  16, -62},    
    qRArm = qRArmTarget2*RAD_TO_DEG,--{140, -0,  5, -98, -42, -16, -62},     
    qLLeg = {25.00,    2.30,  -75.00,   10.00,  -30.00,   -0.00},
    qRLeg = {-15.00,   -2.30,  -75.00,   10.00,  -30.00,    0.00},
    qWaist = {0,-30},
    duration = 3.0*slow_down_factor,
  },
  {  
    qLArm = qLArmTarget2*RAD_TO_DEG,--{140,  0,  5, -98,  42,  16, -62},    
    qRArm = qRArmTarget2*RAD_TO_DEG,--{140, -0,  5, -98, -42, -16, -62},     
    qLLeg = {30.00,    2.30,  -88.00,   10.00,  -30.00,   -0.00},
    qRLeg = {-15.00,   -2.30,  -88.00,   10.00,  -30.00,    0.00},
    qWaist = {0,-30},
    duration = 3.0*slow_down_factor,
  },
  {  
    qLArm = qLArmTarget2*RAD_TO_DEG,--{140,  0,  5, -98,  42,  16, -62},    
    qRArm = qRArmTarget2*RAD_TO_DEG,--{140, -0,  5, -98, -42, -16, -62},     
    qLLeg = {30.00,    2.30,  -88.00,   10.00,  -30.00,   -0.00},
    qRLeg = {-15.00,   12.30,  -88.00,   3.00,  45.00,    0.00},
    qWaist = {30,-30},
    duration = 3.0*slow_down_factor,
  },
  {  
    qLArm = qLArmTarget2*RAD_TO_DEG,--{140,  0,  5, -98,  42,  16, -62},    
    qRArm = qRArmTarget2*RAD_TO_DEG,--{140, -0,  5, -98, -42, -16, -62},     
    qLLeg = {30.00,    32.00,  -88.00,   10.00,  -30.00,   -0.00},
    qRLeg = {-15.00,   12.30,  -88.00,   80.00,  45.00,    0.00},
    qWaist = {30,-30},
    duration = 5.0*slow_down_factor,
  },
  {  
    qLArm = qLArmTarget2*RAD_TO_DEG,--{140,  0,  5, -98,  42,  16, -62},    
    qRArm = qRArmTarget2*RAD_TO_DEG,--{140, -0,  5, -98, -42, -16, -62},     
    qLLeg = {30.00,    32.00,  -88.00,   10.00,  -30.00,   -0.00},
    qRLeg = {-15.00,   12.30,  -90.00,   120.00,  45.00,    0.00},
    qWaist = {30,-30},
    duration = 5.0*slow_down_factor,
  },
  {
    qRArm = qRArmTarget3*RAD_TO_DEG,--{140, -0,  5, -98, -42, -16, -62},     
    qWaist = {-60,-40},
    duration = 3.0*slow_down_factor,
  },
  {
    qRArm = qRArmTarget3*RAD_TO_DEG,--{140, -0,  5, -98, -42, -16, -62},     
    --qRLeg = {-15.00,   45.00,  -90.00,   120.00,  32.00,    0.00},
    qWaist = {-60,-50},
    duration = 3.0*slow_down_factor,
  },
  {
    qLLeg = {30.00,    40.00,  -88.00,   30.00,  30.00,   -0.00},
    qRLeg = {-45.00,   12.30,  -90.00,   120.00,  22.00,    0.00},
    
    duration = 3.0*slow_down_factor,
  },
  {
    qLLeg = {30.00,    40.00,  -88.00,   50.00,  40.00,   -0.00},
    qRLeg = {-45.00,   12.30,  -90.00,   120.00,  22.00,    0.00},
    
    duration = 3.0*slow_down_factor,
  },
  --[[
  {
    qLArm = {140, -0,  5, -98, -42, -16, -62},
    qLLeg = {30.00,    40.00,  -88.00,   90.00,  80.00,   -0.00},
    qRLeg = {-15.00,   12.30,  -90.00,   120.00,  0.00,    0.00},
    
    duration = 3.0*slow_down_factor,
  },
  --]]
  --[[
  {  
    qLArm = qLArmTarget2*RAD_TO_DEG,--{140,  0,  5, -98,  42,  16, -62},    
    qRArm = qRArmTarget2*RAD_TO_DEG,--{140, -0,  5, -98, -42, -16, -62},     
    qLLeg = {30.00,    -25.00,  -88.00,   10.00,  -30.00,   -0.00},
    qRLeg = {-15.00,   -25.30,  -88.00,   3.00,  45.00,    0.00},
    qWaist = {-45,-30},
    duration = 3.0*slow_down_factor,
  },
  {  
    qLArm = qLArmTarget2*RAD_TO_DEG,--{140,  0,  5, -98,  42,  16, -62},    
    qRArm = qRArmTarget2*RAD_TO_DEG,--{140, -0,  5, -98, -42, -16, -62},     
    qLLeg = {30.00,    -25.00,  -88.00,   10.00,  -30.00,   -0.00},
    qRLeg = {-15.00,   -25.30,  -88.00,   80.00,  45.00,    0.00},
    qWaist = {-45,-30},
    duration = 5.0*slow_down_factor,
  },
  {  
    qLArm = qLArmTarget2*RAD_TO_DEG,--{140,  0,  5, -98,  42,  16, -62},    
    qRArm = qRArmTarget2*RAD_TO_DEG,--{140, -0,  5, -98, -42, -16, -62},     
    qLLeg = {30.00,    -25.00,  -88.00,   10.00,  -30.00,   -0.00},
    qRLeg = {-15.00,   -25.30,  -90.00,   120.00,  45.00,    0.00},
    qWaist = {-45,-30},
    duration = 5.0*slow_down_factor,
  }
  --]]
}

local stage
local DEG_TO_RAD = math.pi/180
---------------------------
-- State machine methods --
---------------------------
function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_start = t_entry
  
  qLArm1 = vector.new(Body.get_larm_command_position())  
  qRArm1 = vector.new(Body.get_rarm_command_position())  
  qLLeg1 = vector.new(Body.get_lleg_command_position())  
  qRLeg1 = vector.new(Body.get_rleg_command_position())  
  qWaist1 = vector.new(Body.get_waist_command_position())  

--print(unpack(vector.new(qLArm1)/DEG_TO_RAD))

  stage = 0  
  hcm.set_state_proceed(1)
  head_ch:send'teleop'
end

function state.update()
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_start
print("update")
  print(stage)
  
  print(#keyframe)
--  if stage > #keyframe then return "done" end -- Task done! ( current #keyframe=12 )

  if stage==0 then -- Initial 
    local rpy = Body.get_rpy()
    if math.abs(rpy[1])<45*math.pi/180 and
      math.abs(rpy[2])<45*math.pi/180 then
    --check imu angle.....
      return 'done' end
  end

  hcm.set_motion_headangle({0, 30*math.pi/180})

  if stage>0 and stage < (#keyframe+1) then    
    local ph = math.max(0,math.min(1, t_diff/keyframe[stage].duration))
    
    Body.set_larm_command_position( (1-ph)*qLArm0 + ph*qLArm1 )

    --hack to fix weird motion
  
   -- local qRArmCurrent={}
   -- local qLArmCurrent={}
   -- for i=1,7 do
    --  qRArmCurrent[i] = qRArm0[i] + util.mod_angle(qRArm1[i]-qRArm0[i])*ph
    --  qLArmCurrent[i] = qLArm0[i] + util.mod_angle(qLArm1[i]-qLArm0[i])*ph
   -- end


    Body.set_rarm_command_position((1-ph)*qRArm0 + ph * qRArm1)
--    Body.set_rarm_command_position(qRArmCurrent)
    Body.set_lleg_command_position((1-ph)*qLLeg0 + ph * qLLeg1)
    Body.set_rleg_command_position((1-ph)*qRLeg0 + ph * qRLeg1)
    Body.set_waist_command_position((1-ph)*qWaist0 + ph * qWaist1)
  end

  if stage==0 or (t_diff>keyframe[stage].duration and hcm.get_state_proceed()==1) then

    if stage > 0 and keyframe[stage].stop then hcm.set_state_proceed(0) end
    --if stage > 3 then hcm.set_state_proceed(0) end

    t_start=t
    stage = stage +1
    print(#keyframe)
    if stage > #keyframe then 
      print("DONE")
      return "done" 
    end

    unix.usleep(100000)

    qLArm0 = qLArm1
    qRArm0 = qRArm1
    qLLeg0 = qLLeg1
    qRLeg0 = qRLeg1
    qWaist0 = qWaist1
    if keyframe[stage].qLArm then qLArm1 = vector.new(keyframe[stage].qLArm)*DEG_TO_RAD end
    if keyframe[stage].qRArm then qRArm1 = vector.new(keyframe[stage].qRArm)*DEG_TO_RAD end
    if keyframe[stage].qLLeg then qLLeg1 = vector.new(keyframe[stage].qLLeg)*DEG_TO_RAD end
    if keyframe[stage].qRLeg then qRLeg1 = vector.new(keyframe[stage].qRLeg)*DEG_TO_RAD end
    if keyframe[stage].qWaist then qWaist1 = vector.new(keyframe[stage].qWaist or qWaist0)*DEG_TO_RAD end
  end
--if stage > #keyframe then hcm.set_state_proceed(0) end
end -- walk.update

function state.exit()
  print(state._NAME..' Exit')
  -- TODO: Store things in shared memory?
  hcm.set_motion_headangle({0,0*math.pi/180})
end

return state

