-- Teleop code using old jacobian arm planner
local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlanSJOLD'  
local arm_planner = libArmPlan.new_planner()
local stage
local plan_valid = true

function check_override()
  local override = hcm.get_state_override()
  for i=1,7 do if override[i]~=0 then return true end end
  return false
end

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry   -- Update the time of entry
  t_entry = Body.get_time()
  t_update = t_entry
  arm_planner:set_hand_mass(0,0)
  mcm.set_arm_endpoint_compensation({1,0}) -- compensate for torso movement for only right hand (left arm fixed)
  plan_valid,stage = arm_planner:plan_arm_sequence(Config.armfsm.teleopl.arminit,stage,"arminit")  
end

function state.update()
  if not plan_valid then  
    print("PLANNING ERROR!!!!")
    return "done" end
  local t=Body.get_time()
  local trLArm,trRArm=arm_planner:load_current_condition() 
  if stage=="arminit" then --Turn yaw angles first
    if arm_planner:play_arm_sequence(t) then stage="teleopwait" end
  elseif stage=="teleopwait" then          
    if hcm.get_state_proceed(0)== -1 then       
      plan_valid,stage = arm_planner:plan_arm_sequence(Config.armfsm.teleopl.armuninit,stage,"armuninit")        
      if not plan_valid then
        plan_valid=true
        print("cannot return to initial pose!")        
      end
    elseif hcm.get_state_proceed()==2 then --override
      hcm.set_state_proceed(0)
      local trRArmTarget = hcm.get_hands_right_tr_target()
      local trLArmTarget = hcm.get_hands_left_tr_target()
      local arm_seq = {{'move',nil,trRArmTarget}}   
      plan_valid,stage = arm_planner:plan_arm_sequence(arm_seq,stage,"teleopmove")
      if not plan_valid then plan_valid,stage=true,"teleopwait" end      
    else 
      if check_override() then --Model modification
        local trLArmTarget = vector.new(hcm.get_hands_left_tr_target())
                            +vector.new(hcm.get_state_override())
        hcm.set_state_override({0,0,0,0,0,0,0})
        local arm_seq = {{'move',trLArmTarget,nil}}
        plan_valid,stage = arm_planner:plan_arm_sequence(arm_seq,stage,"teleopmove")
        if not plan_valid then plan_valid,stage=true,"teleopwait" end
      end
    end
  elseif stage=="teleopmove" then 
    if arm_planner:play_arm_sequence(t) then       
      print("Current rarm:",util.print_transform(trLArm,3))
      stage="teleopwait" 
    end
  elseif stage=="armuninit" then     
    if arm_planner:play_arm_sequence(t) then return "done" end    
  end
  hcm.set_state_proceed(0)
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
