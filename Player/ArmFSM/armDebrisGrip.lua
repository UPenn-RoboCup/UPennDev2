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
local lhand_rpy0 = Config.armfsm.debrisgrip.lhand_rpy
local rhand_rpy0 = Config.armfsm.debrisgrip.rhand_rpy

local gripL, gripR = 1,1
local stage
local debugdata

local function get_tool_tr(tooloffset)
  local handrpy = rhand_rpy0
  local tool_model = hcm.get_debris_model()
  local hand_pos = vector.slice(tool_model,1,3) + 
    vector.new({tooloffset[1],tooloffset[2],tooloffset[3]})  
  local tool_tr = {hand_pos[1],hand_pos[2],hand_pos[3],
                    handrpy[1],handrpy[2],handrpy[3] + tool_model[4]}
  return tool_tr
end

local function get_hand_tr(pos)
  return {pos[1],pos[2],pos[3], unpack(rhand_rpy0)}
end

local function check_override()
  local override = hcm.get_state_override()
    for i=1,7 do
    if override[i]~=0 then return true end
  end
  return false
end

local function check_override_support()
  local override = hcm.get_state_override_support()
    for i=1,7 do
    if override[i]~=0 then return true end
  end
  return false
end


local function update_model()
  local trRArmTarget = hcm.get_hands_right_tr_target()
  local trRArm = hcm.get_hands_right_tr()
  local tool_model = hcm.get_debris_model()
  tool_model[1],tool_model[2],tool_model[3], tool_model[4] = 
  tool_model[1] + trRArmTarget[1] - trRArm[1],
  tool_model[2] + trRArmTarget[2] - trRArm[2],
  tool_model[3] + trRArmTarget[3] - trRArm[3],
  tool_model[4] + util.mod_angle(trRArmTarget[6] - trRArm[6])

  hcm.set_debris_model(tool_model)
  print("Tool model:",tool_model[1],tool_model[2],tool_model[3])
  hcm.set_state_proceed(0)
end


function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  
  mcm.set_arm_rhandoffset(Config.arm.handoffset.gripper)
  mcm.set_arm_lhandoffset(Config.arm.handoffset.gripper)



  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  qLArm0 = qLArm
  qRArm0 = qRArm
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

  --Initial arm joint angles after rotating wrist
  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

  arm_planner:set_hand_mass(0,0)
  arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3]) --Lock left hand  

  local bend_seq = {
    {'move',nil,nil,0*Body.DEG_TO_RAD,Config.armfsm.debrisgrip.body_bend}}
  if arm_planner:plan_arm_sequence2(bend_seq) then stage = "benddown" end  

  hcm.set_debris_model(Config.armfsm.debrisgrip.default_model)
--  hcm.set_state_proceed(1)
  hcm.set_state_proceed(0)

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
  
----------------------------------------------------------
--Forward motions
----------------------------------------------------------
  if stage=="benddown" then --Turn yaw angles first    
    if arm_planner:play_arm_sequence(t) then       
      if hcm.get_state_proceed()==1 then 
        arm_planner:set_shoulder_yaw_target(nil,nil) --Lock left hand  
--[[        
        arm_seq={ 
          {'move',nil,Config.armfsm.debrisgrip.arminit[1]},
          {'wrist',nil,Config.armfsm.debrisgrip.arminit[2]},
        }
--]]     
        arm_seq={ 
          {'move',Config.armfsm.debrisgrip.larminit[1],Config.armfsm.debrisgrip.arminit[1]},
          {'wrist',Config.armfsm.debrisgrip.larminit[2],Config.armfsm.debrisgrip.arminit[2]},
        }   
--        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armbacktoinitpos" end
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armreachout" end
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3]) --Lock left hand  
        local bend_seq = {
          {'move',trLArm0,trRArm0,0*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD},
        } --straighten waist
        if arm_planner:plan_arm_sequence(bend_seq) then stage = "done" end  
      end
    end
  elseif stage=="armreachout" then --Turn yaw angles first    
--    if arm_planner:play_arm_sequence(t) then   
    if arm_planner:play_arm_sequence(t) then       
      if hcm.get_state_proceed()==1 then 
--        print("trLArm:",arm_planner.print_transform(trLArm))
        print("trRArm:",arm_planner.print_transform(trRArm))
--        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armup" end
        hcm.set_state_proceed(0)--stop at next step
      elseif hcm.get_state_proceed()==-1 then 
        arm_seq={ 
          {'move',Config.armfsm.debrisgrip.larminit[3],Config.armfsm.debrisgrip.arminit[3]},
          {'wrist',trLArm0,trRArm0},
          {'move',trLArm0,trRArm0},
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "benddown" end
        hcm.set_state_proceed(0)--stop at next step
      elseif check_override() then 
        local trRArmCurrent = hcm.get_hands_right_tr()
        local override = hcm.get_state_override()
        local trRArmTarget = {
          trRArmCurrent[1]+override[1],
          trRArmCurrent[2]+override[2],
          trRArmCurrent[3]+override[3],
          trRArmCurrent[4],
          trRArmCurrent[5],
          trRArmCurrent[6]
        }
        print("trRarm:",arm_planner.print_transform(trRArmTarget))
        hcm.set_state_override({0,0,0,0,0,0,0})        
        local arm_seq = {{'move',nil, trRArmTarget}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armreachout" end
        hcm.set_state_proceed(0)--stop at next step
      elseif check_override_support() then
        local trLArmCurrent = hcm.get_hands_left_tr()
        local override_support = hcm.get_state_override_support()
        local trLArmTarget = {
          trLArmCurrent[1]+override_support[1],
          trLArmCurrent[2]+override_support[2],
          trLArmCurrent[3]+override_support[3],
          trLArmCurrent[4],
          trLArmCurrent[5],
          trLArmCurrent[6]
        }
        print("trLarm:",arm_planner.print_transform(trLArmTarget))
        hcm.set_state_override_support({0,0,0,0,0,0,0})        
        local arm_seq = {{'move',trLArmTarget,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then 
          stage = "armreachout" 
        end
        hcm.set_state_proceed(0)--stop at next step
      end
    end
  elseif stage=="unbenddown" then --Turn yaw angles first        
    if arm_planner:play_arm_sequence(t) then    
    end
  elseif stage=="done" then --Move arm back to holding position
    if arm_planner:play_arm_sequence(t) then    
      return"done"      
    end      

----------------------------------------------------------
--Backward motions motions
----------------------------------------------------------

  elseif stage=="ungrab" then --Ungrip the object
    gripL,doneL = util.approachTol(gripL,0,2,dt)
    gripR,doneR = util.approachTol(gripL,0,2,dt)
    --Body.set_lgrip_percent(gripL*0.8)
    Body.set_rgrip_percent(gripR*0.8)    
--    if doneL then
    if doneR then
      arm_planner:set_hand_mass(0,0)   
      local trRArmTarget2 = get_tool_tr({0,0,0})

      local arm_seq = {{'move',nil, trRArmTarget2}}
      if arm_planner:plan_arm_sequence2(arm_seq) then stage = "touchtool" end
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