local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'  
local arm_planner = libArmPlan.new_planner()
local stage
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
  tool_model[4] + override[6], --yaw
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
  tool_model[4] - override[6], --yaw
  hcm.get_tool_model()
  hcm.set_tool_model(tool_model)
  print( util.color('Tool model:','yellow'), string.format("%.2f %.2f %.2f / %.1f",
        tool_model[1],tool_model[2],tool_model[3],tool_model[4]*180/math.pi ))
  hcm.set_state_proceed(0)
  hcm.set_state_override({0,0,0,0,0,0,0}) 
end
local function confirm_override() hcm.set_state_override({0,0,0,0,0,0,0}) end

local function get_tool_tr()  
  local tool_model = hcm.get_tool_model()
  local hand_pos = vector.slice(tool_model,1,3)  
  local tool_tr = {hand_pos[1],hand_pos[2],hand_pos[3], 0,0,tool_model[4]}
  return tool_tr
end

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry   -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
  arm_planner:set_hand_mass(0,0)
  mcm.set_arm_endpoint_compensation({0,1}) -- compensate for torso movement for only right hand (left arm fixed)
--  arm_planner:set_shoulder_yaw_target(nil,nil)
--  arm_planner:reset_torso_comp(qLArm0,qRArm0)
  plan_valid,stage = arm_planner:plan_arm_sequence(Config.armfsm.teleop.arminit,stage,"arminit")  
end


function state.update()
--  print(state._NAME..' Update' )
  if not plan_valid then  
    print("PLANNING ERROR!!!!")
    return "done" end
  local t  = Body.get_time()
  local dt = t - t_update  
  t_update = t  

  local trLArm,trRArm=arm_planner:load_current_condition()
  --DESIRED WRIST ANGLE: L 90,-90 R -90, 90

  if stage=="arminit" then --Turn yaw angles first
    if arm_planner:play_arm_sequence(t) then 
      hcm.set_tool_model({trRArm[1],trRArm[2],trRArm[3],trRArm[6]})
      stage="teleopwait" 
    end
  elseif stage=="teleopwait" then          
    if hcm.get_state_proceed(0)== -1 then       
      plan_valid,stage = arm_planner:plan_arm_sequence(Config.armfsm.teleop.armuninit,stage,"armuninit")        
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
        hcm.set_hands_right_tr_target(hcm.get_hands_right_tr_target_old())
        plan_valid,stage=true,"teleopwait"
      end
      hcm.set_state_proceed(0)
    else 
      if check_override() then --Model modification
        update_override()
        local arm_seq = {{'move',nil,get_tool_tr()}}
        plan_valid,stage = arm_planner:plan_arm_sequence(arm_seq,stage,"teleopmove")
        if plan_valid then confirm_override()
        else revert_override()
          plan_valid,stage=true,"teleopwait"
        end
      end
    end

  elseif stage=="teleopmove" then 
    if arm_planner:play_arm_sequence(t) then 
      print("Current rarm:",util.print_transform(trRArm,3))
      stage="teleopwait" 
    end
  elseif stage=="armuninit" then 
    --TODO: arm is not going back exactly to the initial position (due to the body movement)    
    if arm_planner:play_arm_sequence(t) then return "done" end    
  end
  hcm.set_state_proceed(0)
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
