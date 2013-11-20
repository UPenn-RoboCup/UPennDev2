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
local lhand_rpy0 = {90*Body.DEG_TO_RAD,0,0}
local rhand_rpy0 = {-90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0}
local trLArm1, trRArm1
local stage

local qLArmInit0,qRArmInit0
local qLArm0, qRArm0
local qWaist

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  qLArm0 = Body.get_larm_command_position()
  qRArm0 = Body.get_rarm_command_position()
  --[[
  qLArm0 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})  
  qRArm0 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  --]]
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

  hcm.set_hands_left_tr(trLArm0)
  hcm.set_hands_right_tr(trRArm0)
  hcm.set_hands_left_tr_target(trLArm0)
  hcm.set_hands_right_tr_target(trRArm0)

  arm_planner:reset_torso_comp(qLArm0, qRArm0)
  arm_planner:save_boundary_condition({qLArm0, qRArm0, qLArm0, qRArm0, {0,0}})
  stage = "wristturn";  
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
    if movearm.setArmJoints(qLArm0,qRArm0,dt, Config.arm.slow_limit) ==1 then       
      stage = "teleopwait"      
    end
  elseif stage=="teleopwait" then       
    
    local qLArm = Body.get_larm_command_position()
    local qRArm = Body.get_rarm_command_position()

    local qLArm1 = Body.get_inverse_larm(qLArm,trLArm0)
    local qRArm1 = Body.get_inverse_rarm(qRArm,trRArm0)

    movearm.setArmJoints(qLArm1,qRArm1,dt)

    if hcm.get_state_proceed()==1 then
      qWaist = Body.get_waist_command_position()
      qWaist = util.approachTol(
        qWaist,
--        vector.new({20*Body.DEG_TO_RAD,0}),
--        vector.new({1*Body.DEG_TO_RAD,0}),
        vector.new({0,79*Body.DEG_TO_RAD}),
        vector.new({0,5*Body.DEG_TO_RAD}),
        dt)
      Body.set_waist_command_position(qWaist)

    elseif hcm.get_state_proceed()==-1 then
      qWaist = Body.get_waist_command_position()
      qWaist = util.approachTol(
        qWaist,
--        vector.new({-20*Body.DEG_TO_RAD,0}),
--        vector.new({1*Body.DEG_TO_RAD,0}),
        vector.new({0,-20*Body.DEG_TO_RAD}),
        vector.new({0,1*Body.DEG_TO_RAD}),        
        dt)
      Body.set_waist_command_position(qWaist)

    end
  elseif stage=="teleopmove" then 
    if arm_planner:play_arm_sequence(t) then 
      stage="teleopwait"
    end
  end
  --hcm.set_state_proceed(0)
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
