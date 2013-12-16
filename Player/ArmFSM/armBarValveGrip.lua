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


local turn_angle1, turn_angle2, wrist_angle
local tight_angle = Config.armfsm.valvebar.handtightangle0
local clearance = Config.armfsm.valvebar.clearance


local function update_override()
  local override = hcm.get_state_override()
  local override_task = hcm.get_state_override_task()
  local valve_model = hcm.get_barvalve_model()

  valve_model={
    valve_model[1] + override[1],
    valve_model[2] + override[2],
    valve_model[3] + override[3],
    valve_model[4], --This is the gripping radius (5cm)
    0, --This should be the radius 
    valve_model[6] + override_task*Config.armfsm.valvebar.turnUnit,  --the target angle
    valve_model[7]--the wrist angle
    }

  hcm.set_barvalve_model(valve_model)
  hcm.set_state_proceed(0)
  print( util.color('Bar Valve model:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f %.1f",
      valve_model[1],valve_model[2],valve_model[3],
      valve_model[6]*180/math.pi,
      valve_model[7]*180/math.pi
         ))
  turn_angle1 = valve_model[6] --Target angle
  wrist_angle = valve_model[7] --Wrist tight angle
end

local function revert_override()
local override = hcm.get_state_override()
  local override_task = hcm.get_state_override_task()
  local valve_model = hcm.get_barvalve_model()

  valve_model={
    valve_model[1] - override[1],
    valve_model[2] - override[2],
    valve_model[3] - override[3],
    valve_model[4], --This is the gripping radius (5cm)
    0, --This should be the radius 
    valve_model[6] - override_task*Config.armfsm.valvebar.turnUnit,  --the target angle
    valve_model[7]--the wrist angle
    }
  hcm.set_barvalve_model(valve_model)
  hcm.set_state_proceed(0)
  turn_angle1 = valve_model[6] --Target angle
  wrist_angle = valve_model[7] --Wrist tight angle
end

local function confirm_override()
  hcm.set_state_override({0,0,0,0,0,0})
  hcm.set_state_override_task(0)  
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_lhandoffset(Config.arm.handoffset.chopstick)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  qLArm0 = qLArm
  qRArm0 = qRArm
  
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, Config.armfsm.valveonearm.arminit[1])  
  qRArm1 = qRArm
  
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

  arm_planner:set_shoulder_yaw_target(nil,qRArm0[3])--unlock left shoulder

--  hcm.set_barvalve_model({0.55,0.20,0.07,   0.05, 0, 70*Body.DEG_TO_RAD })
  hcm.set_barvalve_model(Config.armfsm.valvebar.default_model)

  local valve_model = hcm.get_barvalve_model()
  turn_angle1 = 0 --Initial angle
  turn_angle2 = valve_model[6] --Target angle
  wrist_angle = valve_model[7] --Wrist tight angle

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
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local trLArm = Body.get_forward_larm(qLArm)
  local trRArm = Body.get_forward_rarm(qRArm)  
  
  if stage=="wristturn" then --Turn yaw angles first
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
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
        local trLArmTarget = movearm.getBarValvePositionSingle(0,tight_angle,0)
        local arm_seq = {
          {'wrist',trLArmTarget, nil},
          {'move',trLArmTarget, nil}
        }
        if arm_planner:plan_arm_sequence(arm_seq) then 
          stage="inposition" 
          arm_planner:save_valveparam({0,tight_angle,0,1})       
        end
        hcm.set_state_proceed(0)
      elseif hcm.get_state_proceed()==-1 then               
        local arm_seq = {{'move',trLArm1, nil}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage="wristturn" end
      end
    end
  
  elseif stage=="inposition" then 
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then --Try turn 15 degree        
        local valve_seq={          
          {'barvalve',0,wrist_angle,0,1},
          {'barvalve',turn_angle1,wrist_angle,0,1},
        }
        if arm_planner:plan_arm_sequence(valve_seq) then stage="valveturn" end 
        hcm.set_state_proceed(0)


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
      elseif hcm.get_state_proceed()==3 then --teleop signal
        update_override()
        local valve_seq={{'barvalve',turn_angle1,tight_angle,0,1}}
        if arm_planner:plan_arm_sequence(valve_seq) then 
          stage="inposition"           
        else revert_override() end
        confirm_override()
      end
    end
  
  elseif stage=="valveturn" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==-1 then
        local valve_seq={{'barvalve',turn_angle1,tight_angle,0,1}}
        if arm_planner:plan_arm_sequence(valve_seq) then 
          stage="inposition" 
        end
        hcm.set_state_proceed(0)
      elseif hcm.get_state_proceed()==3 then --OVERRIDE
        print("here")
        update_override()
        print("angle:",turn_angle1)
        local valve_seq={{'barvalve',turn_angle1,wrist_angle,0,1}}
        if arm_planner:plan_arm_sequence(valve_seq) then 
          print("done")
          stage="valveturn" 
          confirm_override()
        else
          revert_override()
        end
        confirm_override()
      end
    end  
  elseif stage=="armbacktoinitpos" then 
    if arm_planner:play_arm_sequence(t) then return "done" end
  end
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state