local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()

--Door opening state using HOOK
local handle_clearance = vector.new({0,0,-0.05})
local lhand_rpy0 = {0*Body.DEG_TO_RAD,0,0}
local rhand_rpy0 = {-0*Body.DEG_TO_RAD,0,0}

local rhand_rpy0 = {-0*Body.DEG_TO_RAD,89*Body.DEG_TO_RAD,0}

local trLArm0, trRArm0, trLArm1, trRArm1, qLArm0, qRarm0
local stage

local qLArmInit0,qRArmInit0

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

  
  arm_planner:set_shoulder_yaw_target(nil,nil)
  
  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})  
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

  hcm.set_hands_left_tr(trLArm1)
  hcm.set_hands_right_tr(trRArm1)
  hcm.set_hands_left_tr_target(trLArm1)
  hcm.set_hands_right_tr_target(trRArm1)
  
  local trLArm05 = {0,0,0, unpack(lhand_rpy0)}
  local trRArm05 = {0,0,0, unpack(rhand_rpy0)}
  trLArm05[2] = trLArm1[2]
  trRArm05[2] = trRArm1[2]

  local wrist_seq = {
    {'wrist',trLArm05, trRArm05},
    {'wrist',trLArm1, trRArm1},
  }
  if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "wristturn" end
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
  
  if stage=="wristturn" then --Turn yaw angles first
    if arm_planner:play_arm_sequence(t) then stage="teleopwait" end
  elseif stage=="teleopwait" then       
    if hcm.get_state_proceed(0)==2 then --teleop signal
      local qLArm = Body.get_larm_command_position()
      print(unpack(vector.new(qLArm)*Body.RAD_TO_DEG))
      local trLArmTarget=  hcm.get_hands_left_tr_target()
      local trRArmTarget=  hcm.get_hands_right_tr_target()
      local arm_seq = {{'move',trLArmTarget, trRArmTarget}}      
      if arm_planner:plan_arm_sequence2(arm_seq) then 
        stage="teleopmove" 
        hcm.set_hands_left_tr(trLArmTarget)
        hcm.set_hands_right_tr(trRArmTarget)
      end
    elseif hcm.get_state_proceed(0)== -1 then 
      arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])
      local arm_seq = {{'move',trLArm1, trRArm1}}
      if arm_planner:plan_arm_sequence2(arm_seq) then stage="armposreset" end
    end
  elseif stage=="teleopmove" then 
    if arm_planner:play_arm_sequence(t) then 
      stage="teleopwait"
    end
  elseif stage=="armposreset" then 
    if arm_planner:play_arm_sequence(t) then 
      local wrist_seq = {{'wrist',trLArm0, trRArm0}}
      if arm_planner:plan_arm_sequence2(wrist_seq) then stage="wristreset" end
    end
  elseif stage=="wristreset" then 
    if arm_planner:play_arm_sequence(t) then return "done" end
  end
  hcm.set_state_proceed(0)
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
