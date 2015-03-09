local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()

local handle_clearance = vector.new({0,0,-0.05})
local lhand_rpy0 = {0,0,45*DEG_TO_RAD}
local rhand_rpy0 = {0,0,45*DEG_TO_RAD}

local trLArm0, trRArm0, trLArm1, trRArm1, qLArm0, qRarm0
local stage

local qLArmInit0,qRArmInit0
local plan_valid = true





function check_override()
  local override = hcm.get_state_override()
  for i=1,7 do if override[i]~=0 then return true end end
  return false
end

local function update_override()
  local override = hcm.get_state_override()
  local tool_model = hcm.get_tool_model()
  tool_model[1],tool_model[2],tool_model[3], tool_model[4] = 
  tool_model[1] + override[1],
  tool_model[2] + override[2],
  tool_model[3] + override[3],
  tool_model[4] + override[6]*5*DEG_TO_RAD, --yaw
  hcm.get_tool_model()
  hcm.set_tool_model(tool_model)
  print( util.color('Tool model:','yellow'), string.format("%.2f %.2f %.2f / %.1f",
        tool_model[1],tool_model[2],tool_model[3],tool_model[4]*180/math.pi ))
  hcm.set_state_proceed(0)
end

local function revert_override()
  print("revert")
  local override = hcm.get_state_override()
  local tool_model = hcm.get_tool_model()
  tool_model[1],tool_model[2],tool_model[3], tool_model[4] = 
  tool_model[1] - override[1],
  tool_model[2] - override[2],
  tool_model[3] - override[3],
  tool_model[4] - override[6]*5*DEG_TO_RAD, --yaw
  hcm.get_tool_model()
  hcm.set_tool_model(tool_model)
  print( util.color('Tool model:','yellow'), string.format("%.2f %.2f %.2f / %.1f",
        tool_model[1],tool_model[2],tool_model[3],tool_model[4]*180/math.pi ))
  hcm.set_state_proceed(0)
  hcm.set_state_override({0,0,0,0,0,0,0}) 
end
local function confirm_override() hcm.set_state_override({0,0,0,0,0,0,0}) end

local function get_tool_tr()
  local handrpy = rhand_rpy0
  local tool_model = hcm.get_tool_model()
  local hand_pos = vector.slice(tool_model,1,3)  
  local tool_tr = {hand_pos[1],hand_pos[2],hand_pos[3], handrpy[1],handrpy[2],tool_model[4]}
--  print("hand transform:",util.print_transform(tool_tr))                    
  return tool_tr
end



function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry


  mcm.set_arm_rhandoffset(Config.arm.handoffset.gripper3)
  mcm.set_arm_lhandoffset(Config.arm.handoffset.gripper3)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  qLArm0 = qLArm
  qRArm0 = qRArm
  
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

  arm_planner:set_hand_mass(0,0)
  mcm.set_arm_endpoint_compensation({0,1}) -- compensate for torso movement for only right hand (left arm fixed)
  arm_planner:set_shoulder_yaw_target(nil,nil)
  
  
--[[  
  hcm.set_hands_left_tr(trLArm1)
  hcm.set_hands_right_tr(trRArm1)
  hcm.set_hands_left_tr_target(trLArm1)
  hcm.set_hands_right_tr_target(trRArm1)
  
  --local wrist_seq = {{'wrist',trLArm1, trRArm1}}
 hcm.set_tool_model({trRArm1[1],trRArm1[2],trRArm1[3],0})
--]]

  plan_valid,stage = arm_planner:plan_arm_sequence(Config.armfsm.teleop.arminit,stage,"wristturn")  
end


function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  if not plan_valid then 
    print("PLANNING ERROR!!!!")
    return "done" end
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  local cur_cond = arm_planner:load_boundary_condition()
  local trLArm = Body.get_forward_larm(cur_cond[1])
  local trRArm = Body.get_forward_rarm(cur_cond[2])  

  
  if stage=="wristturn" then --Turn yaw angles first
    if arm_planner:play_arm_sequence(t) then 
      hcm.set_tool_model({trRArm[1],trRArm[2],trRArm[3],trRArm[6]})
      stage="teleopwait" 
    end
  elseif stage=="teleopwait" then          
    if hcm.get_state_proceed(0)== -1 then       
      plan_valid,stage = arm_planner:plan_arm_sequence(
        Config.armfsm.teleop.armuninit,stage,"armposreset")        
      if not plan_valid then
        print("cannot return to initial pose!")
        plan_valid=true
      end
    elseif hcm.get_state_proceed()==2 then --override
      local trRArmTarget = hcm.get_hands_right_tr_target()
      local trLArmTarget = hcm.get_hands_left_tr_target()
      local arm_seq = {{'move',nil,trRArmTarget}}   
      plan_valid,stage = arm_planner:plan_arm_sequence(arm_seq,stage,"teleopmove")
      if plan_valid then           
        hcm.set_tool_model({trRArmTarget[1],trRArmTarget[2],trRArmTarget[3],trRArmTarget[6]})
      else
        plan_valid=true
      end
      hcm.set_state_proceed(0)
    else 
      if check_override() then --Model modification
        update_override()        
        local arm_seq = {{'move',nil,get_tool_tr()}}   
        plan_valid,stage = arm_planner:plan_arm_sequence(arm_seq,stage,"teleopmove")
        if plan_valid then           
          confirm_override()
        else revert_override() 
          plan_valid,stage=true,"teleopwait"
        end
      end
    end


  elseif stage=="teleopmove" then 
    if arm_planner:play_arm_sequence(t) then 
      print("Current rarm:",util.print_transform(trRArm))
      stage="teleopwait" 
    end
  elseif stage=="armposreset" then 
    if arm_planner:play_arm_sequence(t) then 
      local wrist_seq = {{'wrist',trLArm0, trRArm0}}
      arm_planner:set_shoulder_yaw_target(nil, Config.arm.ShoulderYaw0[2])   
      plan_valid,stage = arm_planner:plan_arm_sequence(wrist_seq,stage,"wristreset")      
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
