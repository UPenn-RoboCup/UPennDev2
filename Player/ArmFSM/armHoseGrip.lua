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

local function get_model_tr(offset)
  local handrpy = rhand_rpy0
  local model = hcm.get_hose_model()
  offset = offset or {0,0,0}
  local hand_pos = vector.slice(model,1,3) + 
    vector.new({offset[1],offset[2],offset[3]})  
  local tr = {hand_pos[1],hand_pos[2],hand_pos[3],
      handrpy[1],handrpy[2],handrpy[3] + model[4]}
  return tr
end

local function get_hand_tr(pos)
  return {pos[1],pos[2],pos[3], unpack(rhand_rpy0)}
end

local function update_model()
  local trRArmTarget = hcm.get_hands_right_tr_target()
  local trRArm = hcm.get_hands_right_tr()
  local model = hcm.get_hose_model()
  model[1],model[2],model[3], model[4] = 
    model[1] + trRArmTarget[1] - trRArm[1],
    model[2] + trRArmTarget[2] - trRArm[2],
    model[3] + trRArmTarget[3] - trRArm[3],
    model[4] + util.mod_angle(trRArmTarget[6] - trRArm[6])
  hcm.set_hose_model(model)
end


function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_handoffset(Config.arm.handoffset.gripper)

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
  arm_planner:set_shoulder_yaw_target(qLArm0[3], nil) --Lock left hand
  local wrist_seq = {{'wrist',nil,trRArm1}}
  if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "wristyawturn" end  
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
    gripR,doneR = util.approachTol(gripR,1,2,dt)  --Close gripper
    Body.set_rgrip_percent(gripR*0.8)
    if arm_planner:play_arm_sequence(t) then       
      if hcm.get_state_proceed()==1 then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],nil)
        trRArmTarget1 = get_hand_tr(Config.armfsm.hosegrip.arminit[2])
        trRArmTarget2 = get_hand_tr(Config.armfsm.hosegrip.arminit[3])
        local arm_seq = {{'move',nil,trRArmTarget1},{'move',nil,trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armup" end
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3]) 
        local wrist_seq = {{"wrist",nil,trRArm0}}
        if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "armbacktoinitpos" end  
      end
    end
  elseif stage=="armup" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],nil)
        local trRArmTarget1 = get_model_tr(Config.armfsm.hosegrip.clearance)        
        local arm_seq = {{'move',nil, trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "reachout" end
      elseif hcm.get_state_proceed()==-1 then 
        trRArmTarget1 = get_hand_tr(Config.armfsm.hosegrip.arminit[2])
        local arm_seq = {{'move',nil,trRArmTarget1},{'move',nil,trRArm1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "wristyawturn" end  
      end
    end 
    
  elseif stage=="reachout" then --Move arm to the gripping position
    gripR,doneR = util.approachTol(gripR,0,2,dt)  --Open gripper
    Body.set_rgrip_percent(gripR*0.8)
    if arm_planner:play_arm_sequence(t) and doneR then 
      if hcm.get_state_proceed()==1 then 
        local trRArmTarget2 = get_model_tr({0,0,0})        
        local arm_seq = {{'move',nil, trRArmTarget2}}     
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "touchtool" end        
      elseif hcm.get_state_proceed() == -1 then        
        local trRArmTarget1 = get_hand_tr(Config.armfsm.hosegrip.arminit[3])
        local arm_seq={{'move',nil,trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armup" end
      elseif hcm.get_state_proceed() == 2 then --Model modification
        update_model()        
        arm_planner:set_hand_mass(0,0)
        local trRArmTarget1 = get_model_tr(Config.armfsm.toolgrip.tool_clearance)
        local arm_seq = {{'move',nil,trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "reachout" end
      end
    end
  elseif stage=="touchtool" then --Move arm to the gripping position
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
--        arm_planner:set_hand_mass(0,1)
        local trRArmTarget2 = get_model_tr({0,0,0})
        local arm_seq = {{'move',nil,trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "grab" end
      elseif hcm.get_state_proceed() == -1 then 
        arm_planner:set_hand_mass(0,0)
        local trRArmTarget1 = get_model_tr(Config.armfsm.toolgrip.tool_clearance)
        local arm_seq = {{'move',nil, trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "reachout" end
      elseif hcm.get_state_proceed() == 2 then 
        arm_planner:set_hand_mass(0,0)
        update_model()        
        local trRArmTarget2 = get_model_tr({0,0,0})        
        local arm_seq = {{'move',nil,trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "touchtool" end
      end
    end

  elseif stage=="grab" then --Grip the object   
    gripR,doneR = util.approachTol(gripR,1,2,dt)
    Body.set_rgrip_percent(gripR*0.8)
    if doneR then stage = "torsobalance" end
  elseif stage=="torsobalance" then
    if arm_planner:play_arm_sequence(t) then    
      if hcm.get_state_proceed()==1 then        
        arm_planner:set_hand_mass(0,2)   
        local trRArmTarget3 = get_model_tr(Config.armfsm.toolgrip.tool_liftup)
        local arm_seq = {{'move',nil, trRArmTarget3}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "lift" end
      elseif hcm.get_state_proceed()==-1 then stage="ungrab" 
      elseif hcm.get_state_proceed() == 2 then --Model modification
        update_model()        
        local trRArmTarget2 = get_model_tr({0,0,0})
        local arm_seq = {{'move',nil,trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "torsobalance" end
      end
    end
  elseif stage=="lift" then
    if arm_planner:play_arm_sequence(t) then    
      if hcm.get_state_proceed()==1 then        
        local trRArmTarget4 = get_model_tr(Config.armfsm.toolgrip.tool_liftuppull)
        local trRArmTarget5 = get_hand_tr(Config.armfsm.toolgrip.armhold[1])
        arm_planner:set_hand_mass(0,2)   
        local arm_seq = {{'move',nil,trRArmTarget4},{'move',nil,trRArmTarget5}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "liftpull" end
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_hand_mass(0,1)   
        local trRArmTarget3 = get_model_tr({0,0,0})
        local arm_seq = {{'move',nil,trRArmTarget3}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "torsobalance" end
      elseif hcm.get_state_proceed() == 2 then --Model modification
        update_model()        
        local trRArmTarget2 = get_model_tr(Config.armfsm.toolgrip.tool_liftup)
        local arm_seq = {{'move',nil,trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "lift" end
      end
    end
  elseif stage=="liftpull" then --Move arm back to holding position
    if arm_planner:play_arm_sequence(t) then    
      stage = "pulldone"
      print("SEQUENCE DONE")
      return"hold"      
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
      local trRArmTarget2 = get_model_tr({0,0,0})
      local arm_seq = {{'move',nil, trRArmTarget2}}
      if arm_planner:plan_arm_sequence2(arm_seq) then stage = "reachout" end
    end  
  elseif stage=="armbacktoinitpos" then 
    if arm_planner:play_arm_sequence(t) then return "done" end
  end

  hcm.set_state_proceed(0)
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