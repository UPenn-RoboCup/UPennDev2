local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()

--Door opening state using HOOK



local dqArmMax = Config.arm.slow_limit
local dpArmMax = Config.arm.linear_slow_limit
local dDoorAngleMax = 1*math.pi/180

local stage = 1;
local hinge_pos={}
local door_r,grip_offset_x,door_yaw,door_yaw1=0,0,0,0
local door_hand = 0

local handle_clearance = vector.new({0,0,-0.05})
local handle_pullup = vector.new({0,0,0.08})

local qLArmTarget, qRArmTarget


local shoulderYaw = -6.6*Body.DEG_TO_RAD

local plan_failed = false


local trArmTarget    
local trLArm, trRArm

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  --close gripper
  Body.set_lgrip_percent(0.8)
  Body.set_rgrip_percent(0.8)
  --Read door model from shm

  local door_model = hcm.get_door_model()
  door_hand = door_model[1]  
  hinge_pos = vector.slice(door_model,2,4)  
  door_r = door_model[5]
  grip_offset_x = door_model[6]
  door_yaw = hcm.get_door_yaw()    
  door_yaw_target = hcm.get_door_yaw_target()  

  door_yaw = 0

  grip = 0

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  local trLArm = Body.get_forward_larm(qLArm)
  local trRArm = Body.get_forward_rarm(qRArm)  

  --First init jangles (wrist straight)
  local lhand_rpy0 = {90*Body.DEG_TO_RAD,0,0}

  local rhand_rpy0 = {-90*Body.DEG_TO_RAD,0,0}



  qRArm0 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  qRArm0[6] = 0,0

  --Second init jangles (wrist bent)  
  qLArm1 = qLArm
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})

  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  
  

--The door we have: hinge height 93.98
--door r: 86.36

  --Right hand pull testing 
  door_hand = 0  --0 for right, 1 for left
  hinge_pos = vector.new({0.55,-1.21,0.01})

  hinge_pos = vector.new({0.55,-1.21,-0.10})

--  hinge_pos = vector.new({0.55,-1.21,-0.05})
  

  door_r = 0.86
  grip_offset_x = -0.05
  door_yaw=0
  door_yaw_target = 30*math.pi/180
 
--[[
  --Right hand push testing 
  door_hand = 0  --0 for right, 1 for left
  hinge_pos = vector.new({0.45,-1.21,-0.05})  
  door_r = 0.86
  grip_offset_x = -0.05
  door_yaw_target = -30*math.pi/180
--]]

--[[
  local trRArmTarget1 = movearm.getDoorHandlePosition(hinge_pos+handle_clearance, door_r, door_yaw, grip_offset_x, door_hand)
  local trRArmTarget2 = movearm.getDoorHandlePosition(hinge_pos, door_r, door_yaw, grip_offset_x,door_hand)
  local trRArmTarget3 = movearm.getDoorHandlePosition(hinge_pos+handle_pulldown, door_r, door_yaw, grip_offset_x,door_hand)

  local trRArmTarget4 = movearm.getDoorHandlePosition(
    hinge_pos+handle_pulldown, door_r, door_yaw+10*Body.DEG_TO_RAD, grip_offset_x,door_hand)
  local trRArmTarget5 = movearm.getDoorHandlePosition(
    hinge_pos+handle_pulldown, door_r, door_yaw+20*Body.DEG_TO_RAD, grip_offset_x,door_hand)
--]]

  local trRArmTarget1 = movearm.getDoorHandlePosition(hinge_pos+handle_clearance, door_r, door_yaw, grip_offset_x, rhand_rpy0)
  local trRArmTarget2 = movearm.getDoorHandlePosition(hinge_pos+handle_pullup, door_r, door_yaw, grip_offset_x, rhand_rpy0)
  local trRArmTarget3 = movearm.getDoorHandlePosition(
    hinge_pos+handle_pullup, door_r, door_yaw+5*Body.DEG_TO_RAD, grip_offset_x, rhand_rpy0)


  trRArmTarget3[5]=0
  trRArmTarget3[6]=0

  local trRArmTarget4 = movearm.getDoorHandlePosition(
    hinge_pos+handle_pullup, door_r, door_yaw+10*Body.DEG_TO_RAD, grip_offset_x, rhand_rpy0)

--[[
  trRArmTarget4[5]=0
  trRArmTarget4[6]=0
--]]

  local trRArmTarget5 = movearm.getDoorHandlePosition(
    hinge_pos+handle_pullup, door_r, door_yaw+15*Body.DEG_TO_RAD, grip_offset_x, rhand_rpy0)
  trRArmTarget5[5]=0
  trRArmTarget5[6]=0


  arm_planner:reset_torso_comp(qLArm1, qRArm1)
  local arm_sequence1 = {
    init={qLArm1,qRArm1,qLArm1, qRArm1, {0,0}},
    mass={0,0},
    armseq={
      {trLArm1, trRArmTarget1},
      {trLArm1, trRArmTarget2},
      {trLArm1, trRArmTarget3},
--      {trLArm1, trRArmTarget4},  
--      {trLArm1, trRArmTarget5},  
    }
  }
  arm_plan1, arm_end1 = arm_planner:plan_arm_sequence(arm_sequence1)
  if not arm_plan1 then plan_failed = true end
  stage = 1;  
end


function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  if plan_failed then return "planfail" end
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  if stage==1 then --Set the arm to grip-ready pose    
    --Turn yaw angles first
    ret = movearm.setArmJoints(qLArm,qRArm0,dt, Config.arm.joint_init_limit) 
    if ret==1 then stage=stage+1 end  
  elseif stage==2 then   
    ret = movearm.setArmJoints(qLArm,qRArm1,dt, Config.arm.joint_init_limit)     
    if ret==1 then 
      stage=stage+1 
      arm_planner:init_arm_sequence(arm_plan1,t)
    end    
  elseif stage==3 then --Move the arm forward using IK now     
    if arm_planner:play_arm_sequence(t) then    
      stage = stage+1             
    end
  end
 
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
