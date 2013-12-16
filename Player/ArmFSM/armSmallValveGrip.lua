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
local lhand_rpy0 = {0,0*Body.DEG_TO_RAD, 0*Body.DEG_TO_RAD}
local rhand_rpy0 = {0,0*Body.DEG_TO_RAD, 0*Body.DEG_TO_RAD}

local trLArm0, trRArm0, trLArm1, trRArm1, qLArm0, qRarm0
local gripL, gripR = 1,1
local stage

local old_valve_model


local angle1




local function update_override()
  local override_old = hcm.get_state_override()
  local override_target = hcm.get_state_override_target()
  local override = vector.new(override_target)- vector.new(override_old)
  local override_task = override[7]

  local valve_model = hcm.get_largevalve_model()

  valve_model[1],valve_model[2],valve_model[3], valve_model[4], valve_model[5], valve_model[6]=
    valve_model[1] + override[1],
    valve_model[2] + override[2],
    valve_model[3] + override[3],
    0, --This should be the radius 
    valve_model[5] + override_task*Config.armfsm.valveonearm.turnUnit,   --this is the init angle
    valve_model[6] --target angle, but we won't use it
    
  hcm.set_largevalve_model(valve_model)

  print( util.color('Valve model:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f",
      valve_model[1],valve_model[2],valve_model[3],
      valve_model[5]*180/math.pi   ))

  angle1 = valve_model[5]
  hcm.set_state_proceed(0)
end

local function revert_override()
  local override_old = hcm.get_state_override()
  local override_target = hcm.get_state_override_target()
  local override = vector.new(override_target)- vector.new(override_old)
  local override_task = override[7]

  local valve_model = hcm.get_largevalve_model()

  valve_model[1],valve_model[2],valve_model[3], valve_model[4], valve_model[5], valve_model[6]=
    valve_model[1] - override[1],
    valve_model[2] - override[2],
    valve_model[3] - override[3],
    0, --This should be the radius 
    valve_model[5] - override_task*Config.armfsm.valveonearm.turnUnit,   --this is the init angle
    valve_model[6] --target angle, but we won't use it
    
  hcm.set_largevalve_model(valve_model)

  print( util.color('Valve model:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f",
      valve_model[1],valve_model[2],valve_model[3],
      valve_model[5]*180/math.pi   ))

  angle1 = valve_model[5]
  hcm.set_state_proceed(0)  
end

local function confirm_override()
  hcm.set_state_override(hcm.get_state_override_target())    
end


function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_lhandoffset(Config.arm.handoffset.chopstick)
  mcm.set_arm_rhandoffset(Config.arm.handoffset.chopstick)

  local cur_cond = arm_planner:load_boundary_condition()
  local qLArm = cur_cond[1]
  local qRArm = cur_cond[2]

  qLArm0, qRArm0 = qLArm, qRArm
  qLArm1, qRArm1 = qLArm, qRArm
  
  
  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, Config.armfsm.valveonearm.arminit[1])  
  
  trLArm0, trRArm0 = Body.get_forward_larm(qLArm0),Body.get_forward_rarm(qRArm0)  
  trLArm1, trRArm1 = Body.get_forward_larm(qLArm1),Body.get_forward_rarm(qRArm1)  

  arm_planner:set_shoulder_yaw_target(nil,qRArm0[3])--unlock left shoulder

  hcm.set_largevalve_model(Config.armfsm.valveonearm.default_model_small)

  hcm.set_state_tstartactual(unix.time()) 
  hcm.set_state_tstartrobot(Body.get_time())

  confirm_override()
  update_override()
  local wrist_seq = {{'wrist',trLArm1, nil}}
  if arm_planner:plan_arm_sequence(wrist_seq) then stage = "wristturn" end
  hcm.set_state_proceed(1)
end


function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update  
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
      if hcm.get_state_proceed()==1 then       
        print("trLArm:",arm_planner.print_transform(trLArm))
        local model = hcm.get_largevalve_model()
        local arm_seq = {
            {'move',Config.armfsm.valveonearm.arminit[1], nil},
            {'move',Config.armfsm.valveonearm.arminit[2], nil},           
            {'move',Config.armfsm.valveonearm.arminit[3], nil},           
          }
        if arm_planner:plan_arm_sequence(arm_seq) then stage="armready" end        
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])
        local wrist_seq = {{'wrist',trLArm0, nil}}
        if arm_planner:plan_arm_sequence(wrist_seq) then stage = "armbacktoinitpos" end
      end
    end
  elseif stage=="armready" then        
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        hcm.set_state_proceed(1)  
        local trLArmTarget = movearm.getLargeValvePositionSingle(angle1,0, 1)
        local arm_seq = {
          {'wrist',trLArmTarget, nil},
          {'move',trLArmTarget, nil}
        }
        if arm_planner:plan_arm_sequence(arm_seq) then 
          stage="inposition" 
          arm_planner:save_valveparam({angle1,0,1,0})
        end
        hcm.set_state_proceed(0)
      end
    end
  
  elseif stage=="inposition" then 
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 

--        print("trLArm:",arm_planner.print_transform(trLArm))
      elseif hcm.get_state_proceed()==-1 then 
        local arm_seq = {
            {'move',Config.armfsm.valveonearm.arminit[4], nil},           
            {'wrist',Config.armfsm.valveonearm.arminit[3], nil},           
            {'move',Config.armfsm.valveonearm.arminit[3], nil},           
            {'move',Config.armfsm.valveonearm.arminit[2], nil},           
            {'move',Config.armfsm.valveonearm.arminit[1], nil},
            {'move',trLArm1, nil},
          }
        if arm_planner:plan_arm_sequence(arm_seq) then stage="wristturn" end
      elseif hcm.get_state_proceed()==3 then --NEW OVERRIDE
        update_override()
        local valve_seq={{'valveonearm',angle1,0,1,0}}
        if arm_planner:plan_arm_sequence(valve_seq) then 
          stage="inposition"         
        else
          revert_override()
        end
        confirm_override()
        hcm.set_state_proceed(0)
      end
    end

  elseif stage=="valverelease" then     
    if arm_planner:play_arm_sequence(t) then 
      local valve_model = hcm.get_largevalve_model()
      valve_model[1] = valve_model[1] + Config.armfsm.valveonearm.clearance 
      hcm.set_largevalve_model(valve_model)
      hcm.set_state_success(1) --Report success
      stage="inposition"
      hcm.set_state_proceed(0)
    end
  elseif stage=="armbacktoinitpos" then 
    if arm_planner:play_arm_sequence(t) then return "done" end
  end  
end

function state.exit()    
  print(state._NAME..' Exit' )
end

return state
