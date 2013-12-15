local state = {}
state._NAME = ...
require'hcm'
require'mcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()

local qLArm0,qRArm0, trLArm0, trRArm0, trLArm1, trARArm1

--Initial hand angle
local lhand_rpy0 = Config.armfsm.hosegrip.lhand_rpy
local rhand_rpy0 = Config.armfsm.hosegrip.rhand_rpy

local gripL, gripR = 1,1
local stage
local debugdata

local function get_model_tr(offset,handrpy)
  local handrpy = handrpy or lhand_rpy0
  local model = hcm.get_hose_model()
  offset = offset or {0,0,0}
  local hand_pos = vector.slice(model,1,3) + 
    vector.new({offset[1],offset[2],offset[3]})  
  local tr = {hand_pos[1],hand_pos[2],hand_pos[3],
      handrpy[1],handrpy[2],handrpy[3] + model[4]}
  return tr
end

local function get_hand_tr(pos,rpy)
  rpy = rpy or lhand_rpy0
  return {pos[1],pos[2],pos[3], unpack(rpy)}
end

local function update_model()
  local trArmTarget = hcm.get_hands_left_tr_target()
  local trArm = hcm.get_hands_left_tr()
  local model = hcm.get_hose_model()
  model[1],model[2],model[3], model[4] = 
    model[1] + trArmTarget[1] - trArm[1],
    model[2] + trArmTarget[2] - trArm[2],
    model[3] + trArmTarget[3] - trArm[3],
    model[4] + util.mod_angle(trArmTarget[6] - trArm[6])
  hcm.set_hose_model(model)
  print("update: model ",model[1],model[2],model[3])
  hcm.set_state_proceed(0)
end


local function update_override()
  local overrideTarget = hcm.get_state_override_target()
  local override = hcm.get_state_override()
  local model = hcm.get_hose_model()
  
  model[1],model[2],model[3],model[4]=
    model[1] + (overrideTarget[1]-override[1]),
    model[2] + (overrideTarget[2]-override[2]),
    model[3] + (overrideTarget[3]-override[3]),
    model[4] + (overrideTarget[4]-override[4])
    
  hcm.set_hose_model(model)

  print( util.color('Hose model:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f",
      model[1],model[2],model[3],
      model[4]*180/math.pi   ))

  hcm.set_state_proceed(0)
end

local function revert_override()
local overrideTarget = hcm.get_state_override_target()
  local override = hcm.get_state_override()
  local model = hcm.get_hose_model()
  
  model[1],model[2],model[3],model[4]=
    model[1] - (overrideTarget[1]-override[1]),
    model[2] - (overrideTarget[2]-override[2]),
    model[3] - (overrideTarget[3]-override[3]),
    model[4] - (overrideTarget[4]-override[4])
    
  hcm.set_hose_model(model)

  print( util.color('Hose model:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f",
      model[1],model[2],model[3],
      model[4]*180/math.pi   ))

  hcm.set_state_proceed(0)end

local function confirm_override()
  local override = hcm.get_state_override()
  hcm.set_state_override_target(override)
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_lhandoffset(Config.arm.handoffset.gripper)
  mcm.set_arm_rhandoffset(Config.arm.handoffset.gripper)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  qLArm0 = qLArm
  qRArm0 = qRArm
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

  --Initial arm joint angles after rotating wrist
  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, Config.armfsm.hosegrip.arminit[1])
  qRArm1 = qRArm
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

  arm_planner:set_hand_mass(0,0)
--  arm_planner:set_shoulder_yaw_target(qLArm0[3], nil) --Lock left hand
  arm_planner:set_shoulder_yaw_target(nil,qRArm0[3]) --Lock left hand
  --local wrist_seq = {{'wrist',nil,trRArm1}}
  local arm_seq = {
    {'wrist',Config.armfsm.hosegrip.arminit[1],nil},
    {'wrist',Config.armfsm.hosegrip.arminit[2],nil},
  }
  if arm_planner:plan_arm_sequence2(arm_seq) then stage = "wristyawturn" end  
  hcm.set_state_proceed(1)

  hcm.set_hose_model(Config.armfsm.hosegrip.default_model)

  debugdata=''   

  hcm.set_state_tstartactual(unix.time())
  hcm.set_state_tstartrobot(Body.get_time())

end

function state.update()
  --  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t   -- Save this at the last update time

  local cur_cond = arm_planner:load_boundary_condition()
  local trLArm = Body.get_forward_larm(cur_cond[1])
  local trRArm = Body.get_forward_rarm(cur_cond[2])  
  

  if stage=="wristyawturn" then --Turn yaw angles first  
    if arm_planner:play_arm_sequence(t) then       
      if hcm.get_state_proceed()==1 then         
        print("trLArm:",arm_planner.print_transform(trLArm))
        local trArmTarget1 = get_model_tr({0,0,0},Config.armfsm.hosegrip.lhand_rpy1)        
        local arm_seq = {{'move',Config.armfsm.hosegrip.arminit[2],nil},
          {'move',trArmTarget1,nil},}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "touchtool" end
        hcm.set_state_proceed(0)
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3]) 
        local wrist_seq = {{"wrist",trLArm0,nil}}
        if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "armbacktoinitpos" end  
      end
    end
  elseif stage=="touchtool" then --Move arm to the gripping position
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then                
        local arm_seq = {
          {'wrist',Config.armfsm.hosegrip.armhold[1],nil},
          {'move',Config.armfsm.hosegrip.armhold[1],nil},
          {'move',Config.armfsm.hosegrip.armhold[2],nil},
        }

        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armretract" end
        hcm.set_state_proceed(0)

      elseif hcm.get_state_proceed() == -1 then 
        local trArmTarget1 = get_model_tr(Config.armfsm.hosegrip.clearance)        
        local trArmTarget2 = get_hand_tr(Config.armfsm.hosegrip.arminit[1])
        local arm_seq = {{'move',trArmTarget1,nil},{'move',trArmTarget2,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "wristyawturn" end

      elseif hcm.get_state_proceed() == 3 then 
        update_override()        
        local trLArmTarget2 = get_model_tr({0,0,0})        
        local arm_seq = {{'move',trLArmTarget2,nil}}     
        if arm_planner:plan_arm_sequence2(arm_seq) then 
          stage = "touchtool" 
          confirm_override()
        else
          revert_override()
        end                        
      end
    end
    
  elseif stage=="armretract" then --Move arm back to holding position
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then                
        local arm_seq={
          {'wrist',Config.armfsm.hosegrip.armhold[3],nil},
          {'move',Config.armfsm.hosegrip.armhold[3],nil},
          {'move',Config.armfsm.hosegrip.armhold[4],nil},
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armretract2" end
        hcm.set_state_proceed(0)
      end
    end
  elseif stage=="armretract2" then --Move arm back to holding position
    if arm_planner:play_arm_sequence(t) then 
      print("trLArm:",arm_planner.print_transform(trLArm))
      stage = "pulldone"
      print("SEQUENCE DONE")
      return"hold"
    end
  elseif stage=="armbacktoinitpos" then 
    if arm_planner:play_arm_sequence(t) then return "done" end
  end

  
end

function state.exit()  
  hcm.set_state_success(1) --Report success
  --Store boundary conditions for future state
  --arm_planner:save_boundary_condition(current_arm_endcond)
  print(state._NAME..' Exit' )
end

local function flush_debugdata()
  local savefile = string.format("Log/debugdata_%s",os.date());
  local debugfile=assert(io.open(savefile,"w")); 
  debugfile:write(debugdata);
  debugfile:flush();
  debugfile:close();  
end

return state
