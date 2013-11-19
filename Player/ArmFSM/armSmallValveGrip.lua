local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()


local lhand_rpy0 = {-20*Body.DEG_TO_RAD,0,0}
local lhand_rpy1 = {90*Body.DEG_TO_RAD,0,0}

local rhand_rpy0 = {20*Body.DEG_TO_RAD,0,0}
local rhand_rpy1 = {-90*Body.DEG_TO_RAD,0,0}

local trLArm0, trRArm0, trLArm1, trRArm1, qLArm0, qRArm0
local stage

local qLArmInit0,qRArmInit0,qLArmInit1,qRArmInit1

local function getTargetTransform(offset, handrpy)
  local smallvalve_model = hcm.get_smallvalve_model()
  offset = offset or {0,0,0}
  handrpy = handrpy or lhand_rpy0
  local trArmTarget = {
    smallvalve_model[1]+offset[1],
    smallvalve_model[2]+offset[2],
    smallvalve_model[3]+offset[3],
    unpack(handrpy)}

  lhand_rpy0 = {smallvalve_model[4],0,0}
  lhand_rpy1 = {smallvalve_model[5],0,0}
  return trArmTarget
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_handoffset(Config.arm.handoffset.chopstick)

  -- Make the human provide the input
  --hcm.set_smallvalve_model({0.50,0.25,0.02,-20*Body.DEG_TO_RAD, 90*Body.DEG_TO_RAD})  

  getTargetTransform() --This updates lhand_rpy0 and lhand_rpy1

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  
  qLArm0 = qLArm
  qRArm0 = qRArm

  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  
  
  local qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})
  local qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})

  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

  arm_planner:reset_torso_comp(qLArm0, qRArm0)
  arm_planner:save_boundary_condition({qLArm0, qRArm0, qLArm0, qRArm0, {0,0}})
  arm_planner:set_hand_mass(0,0)

  arm_planner:set_shoulder_yaw_target(nil,qRArm0[3]) --Lock right shoulder yaw
  local wrist_seq = {{'wrist',trLArm1,nil}}
  if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "wristturn" end  
  hcm.set_state_proceed(1)
end

local function update_model()
  local trLArmTarget = hcm.get_hands_left_tr_target()
  local trLArm = hcm.get_hands_left_tr()
  local smallvalve_model = hcm.get_smallvalve_model()
  smallvalve_model[1],smallvalve_model[2],smallvalve_model[3]=
    smallvalve_model[1] + trLArmTarget[1]-trLArm[1],
    smallvalve_model[2] + trLArmTarget[2]-trLArm[2],
    smallvalve_model[3] + trLArmTarget[3]-trLArm[3]
  hcm.set_smallvalve_model(smallvalve_model)
end


function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  if plan_failed then return "planfail" end
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local trLArm = Body.get_forward_larm(qLArm)

  if stage=="wristturn" then --Turn yaw angles first
    if arm_planner:play_arm_sequence(t) then           
      if hcm.get_state_proceed()==1 then
        print("Current:",arm_planner.print_transform(trLArm))
        local trLArmTarget1 = {0.35,0.25, -0.10, unpack(lhand_rpy0)}      
        local trLArmTarget2 = {0.35,0.25, 0.0, unpack(lhand_rpy0)}
        local arm_seq = {{'move',trLArmTarget1,nil},{'move',trLArmTarget2,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armfront" end      
      elseif hcm.get_state_proceed()==-1 then
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])
        local wrist_seq = {{'wrist',trLArm0,trRArm0 }}    
        if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "armbacktoinitpos" end      
      end
    end
  elseif stage=="armfront" then       
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        local trLArmTarget1 = getTargetTransform()
        trLArmTarget1[1] = 0.35
        local trLArmTarget2 = getTargetTransform({-0.08,0,0})        
        local arm_seq = {{'move',trLArmTarget1,nil},{'move',trLArmTarget2,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armposition" end      
--[[
--High position testing
        local trRArmTarget2 = {0.45,-0.25, 0.30, unpack(rhand_rpy0)}
        local trRArmTarget3 = {0.40,-0.25, 0.45, unpack(rhand_rpy0)}
        local trRArmTarget4 = {0.40,-0.22, 0.57, unpack(rhand_rpy0)}
--]]
      elseif hcm.get_state_proceed()==-1 then
        local trLArmTarget1 = {0.35,0.25, -0.10, unpack(lhand_rpy0)}      
        local arm_seq = {{'move',trLArmTarget1,nil},{'move',trLArm1,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "wristturn" end              
      end
    end
  elseif stage=="armposition" then --Move the arm forward using IK now     
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed(0)==1 then
        local trLArmTarget1 = getTargetTransform()
        local arm_seq = {{'move',trLArmTarget1,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "arminsert" end
      elseif hcm.get_state_proceed(0)==2 then --modification
        update_model()
        local trLArmTarget2 = getTargetTransform({-0.08,0,0})        
        local arm_seq = {{'move',trLArmTarget2,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armposition" end      
      elseif hcm.get_state_proceed(0)==-1 then --go back to center position        
        local trLArmTarget1 = {0.35,0.25, 0.0, unpack(lhand_rpy0)}
        local arm_seq = {{'move',trLArmTarget1,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armfront" end      
      end
    end
  elseif stage=="arminsert" then       
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed(0)==1 then
        local trLArmTarget1 = getTargetTransform({0,0,0},lhand_rpy1)
        local trLArmTarget2 = getTargetTransform({-0.08,0,0},lhand_rpy1)
        local trLArmTarget3 = getTargetTransform({-0.08,0,0},lhand_rpy0)
        local arm_seq = {
          {'move',trLArmTarget1,nil},{'move',trLArmTarget2,nil},{'move',trLArmTarget3,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then 
          hcm.set_state_success(1) --Report success  
          stage = "armposition" 
        end
      elseif hcm.get_state_proceed(0)==2 then   
        update_model()
        local trLArmTarget2 = getTargetTransform()        
        local arm_seq = {{'move',trLArmTarget2,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "arminsert" end      
      elseif hcm.get_state_proceed(0)==-1 then 
        local trLArmTarget2 = getTargetTransform({-0.08,0,0})
        trLArmTarget2[1] = math.max(0.35,trLArmTarget2[1])
        local arm_seq = {{'move',trLArmTarget2,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armposition" end      
      end
    end
  elseif stage=="armbacktoinitpos" then  
    if arm_planner:play_arm_sequence(t) then      
      return "done"
    end
  end
 
 hcm.set_state_proceed(0)
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
