local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()
local T      = require'Transform'

--Initial hand angle
local lhand_rpy0 = {0,0*Body.DEG_TO_RAD, -45*Body.DEG_TO_RAD}
local rhand_rpy0 = {0,0*Body.DEG_TO_RAD, 45*Body.DEG_TO_RAD}
local trLArm0, trRArm0, trLArm1, trRArm1, qLArm0, qRarm0
local stage
local gripL, gripR = 1,1

local qLArmInit0,qRArmInit0

local function getArmWheelPosition(radius_offset, turn_angleL, turn_angleR)
  local wheel   = hcm.get_wheel_model()
  local handle_pos    = vector.slice(wheel,1,3)
  local handle_yaw    = wheel[4]
  local handle_pitch  = wheel[5]
  local handle_radius = wheel[6] + radius_offset

  local lhand_rpy = {0,0*Body.DEG_TO_RAD, -45*Body.DEG_TO_RAD}
  local rhand_rpy = {0,0*Body.DEG_TO_RAD, 45*Body.DEG_TO_RAD}

  --Calculate the hand transforms
  local trHandle = T.eye()
       * T.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * T.rotZ(handle_yaw)       
  
  local trGripL = trHandle
       * T.rotX(turn_angleL)
       * T.trans(0,handle_radius,0)
       * T.transform6D(
          {0,0,0,lhand_rpy[1],lhand_rpy[2],lhand_rpy[3]})  

  local trGripR = trHandle
       * T.rotX(turn_angleR)
       * T.trans(0,-handle_radius,0)
       * T.transform6D(
          {0,0,0,rhand_rpy[1],rhand_rpy[2],rhand_rpy[3]})  
  return T.position6D(trGripL), T.position6D(trGripR)
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  qLArm0 = qLArm
  qRArm0 = qRArm
  
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

  arm_planner:reset_torso_comp(qLArm, qRArm)
  arm_planner:save_boundary_condition({qLArm, qRArm, qLArm, qRArm, {0,0}})
  
  arm_planner:set_shoulder_yaw_target(nil,nil)
  
  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})  
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

  hcm.set_wheel_model({0.36,0.0,0.02,0,0,0.20})

  local wrist_seq = {  armseq={{trLArm1, trRArm1}}  }
  if arm_planner:plan_wrist_sequence(wrist_seq) then   
    stage = "wristturn"
  end
end


function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update  
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local trLArm = Body.get_forward_larm(qLArm)
  local trRArm = Body.get_forward_rarm(qRArm)  
  
  if stage=="wristturn" then --Turn yaw angles first
    gripL,doneL = util.approachTol(gripL,1,2,dt)  --Close gripper
    gripR,doneL = util.approachTol(gripR,1,2,dt)  --Close gripper
    Body.set_lgrip_percent(gripL*0.8)
    Body.set_rgrip_percent(gripR*0.8)
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        local trLArmTarget={0.25,0.15,-0.15, unpack(lhand_rpy0)}
        local trRArmTarget={0.25,-0.15,-0.15, unpack(rhand_rpy0)}
        local arm_seq = {
          mass={0,0},
          armseq={{trLArmTarget, trRArmTarget}}
        }
        if arm_planner:plan_arm_sequence(arm_seq) then stage="armwide" end
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])
        local wrist_seq = {  armseq={{trLArm0, trRArm0}}  }
        if arm_planner:plan_wrist_sequence(wrist_seq) then stage = "armbacktoinitpos" end
      end
    end
  elseif stage=="armwide" then    
    gripL,doneL = util.approachTol(gripL,0,2,dt)  --Open gripper
    gripR,doneL = util.approachTol(gripR,0,2,dt)  --Open gripper
    Body.set_lgrip_percent(gripL*0.8)
    Body.set_rgrip_percent(gripR*0.8)
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        local trLArmTarget, trRArmTarget = getArmWheelPosition(0.08,0,0)
        local arm_seq = {armseq={{trLArmTarget, trRArmTarget}}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage="pregrip" end
      elseif hcm.get_state_proceed()==-1 then         
        local arm_seq = {armseq={{trLArm1, trRArm1}}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage="wristturn" end
      end
    end
  elseif stage=="pregrip" then 
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then --teleop signal
        local trLArmTarget, trRArmTarget = getArmWheelPosition(0,0,0)
        local arm_seq = {armseq={{trLArmTarget, trRArmTarget}}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage="inposition" end
      elseif hcm.get_state_proceed(0)==-1 then
        local trLArmTarget={0.25,0.15,-0.15, unpack(lhand_rpy0)}
        local trRArmTarget={0.25,-0.15,-0.15, unpack(rhand_rpy0)}
        local arm_seq = {armseq={{trLArmTarget, trRArmTarget}}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage="armwide" end
      end
    end
  elseif stage=="inposition" then 
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then --teleop signal
        stage="grip"      
      elseif hcm.get_state_proceed()==-1 then --teleop signal
        local trLArmTarget, trRArmTarget = getArmWheelPosition(0.08,0,0)
        local arm_seq = {armseq={{trLArmTarget, trRArmTarget}}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage="pregrip" end
      end
    end
  elseif stage=="grip" then 
    gripL,doneL = util.approachTol(gripL,1,2,dt)  --Close gripper
    gripR,doneL = util.approachTol(gripR,1,2,dt)  --Close gripper
    Body.set_lgrip_percent(gripL*0.8)
    Body.set_rgrip_percent(gripR*0.8)
  elseif stage=="armbacktoinitpos" then 
    if arm_planner:play_arm_sequence(t) then return "done" end
  end
  hcm.set_state_proceed(0)
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
