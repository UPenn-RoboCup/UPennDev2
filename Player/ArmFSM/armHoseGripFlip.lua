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
  local handrpy = lhand_rpy0
  local model = hcm.get_hose_model()
  offset = offset or {0,0,0}
  local hand_pos = vector.slice(model,1,3) + 
    vector.new({offset[1],offset[2],offset[3]})  
  local tr = {hand_pos[1],hand_pos[2],hand_pos[3],
      handrpy[1],handrpy[2],handrpy[3] + model[4]}
  return tr
end

local function get_hand_tr(pos)
  return {pos[1],pos[2],pos[3], unpack(lhand_rpy0)}
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
  hcm.set_state_proceed(0)
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

print("qLArm:",arm_planner.print_jangle(qLArm))

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
  arm_planner:set_shoulder_yaw_target(nil,qRArm0[3]) --Lock left hand
    
  hcm.set_state_proceed(1)

  hcm.set_hose_model(Config.armfsm.hosegrip.default_model)

  debugdata=''   

  hcm.set_state_tstartactual(unix.time())
  hcm.set_state_tstartrobot(Body.get_time())

  stage = "shoulderup1"
end

function state.update()
  --  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t   -- Save this at the last update time

  local cur_cond = arm_planner:load_boundary_condition()
  local trLArm = Body.get_forward_larm(cur_cond[1])
  local trRArm = Body.get_forward_rarm(cur_cond[2])  


  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  
  if stage=="shoulderup1" then
    local qLArmNew,done = util.approachTol(qLArm,
      Config.armfsm.hosegrip.armflip[1],
      Config.arm.slow_limit,dt)
    movearm.setArmJoints(qLArmNew,qRArm,dt)
    if done then stage="shoulderup2" end
  elseif stage=="shoulderup2" then
    local qLArmNew,done = util.approachTol(qLArm,
      Config.armfsm.hosegrip.armflip[2],
      Config.arm.slow_limit,dt)
    movearm.setArmJoints(qLArmNew,qRArm,dt)
    if done then stage="shoulderup3" end
  elseif stage=="shoulderup3" then
    local qLArmNew,done = util.approachTol(qLArm,
      Config.armfsm.hosegrip.armflip[3],
      Config.arm.slow_limit,dt)
    movearm.setArmJoints(qLArmNew,qRArm,dt)
    if done then stage="shoulderup4" end
  elseif stage=="shoulderup4" then
    local qLArmNew,done = util.approachTol(qLArm,
      Config.armfsm.hosegrip.armflip[4],
      Config.arm.slow_limit,dt)
    movearm.setArmJoints(qLArmNew,qRArm,dt)    
    if done then 
      arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3]) --Lock both shoulder
      arm_planner:reset_torso_comp(qLArm,qRArm)  --HACK
      local wrist_seq = {{'wrist',trLArm1,nil}}

      print("trL:",arm_planner.print_transform(
        Body.get_forward_larm(Config.armfsm.hosegrip.armflip[4])))
      if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "wristyawturn" end  
    end
  elseif stage=="wristyawturn" then --Turn yaw angles first    
    gripL,doneL = util.approachTol(gripL,0,2,dt)  --open gripper
    Body.set_lgrip_percent(gripL*0.8)
    if arm_planner:play_arm_sequence(t) then       
      if hcm.get_state_proceed()==1 then 
        arm_planner:set_shoulder_yaw_target(nil,qRArm0[3])
--      print("trL:",arm_planner.print_transform(trLArm))
        local trLArmTarget1 = Config.armfsm.hosegrip.arminit[1]
        local trLArmTarget2 = Config.armfsm.hosegrip.arminit[2]
        local arm_seq = {{'move',trLArmTarget1,nil},{'move',trLArmTarget2,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armready" end
      elseif hcm.get_state_proceed()==-1 then 

      end
    end
  elseif stage=="armready" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        local trLArmTarget1 = get_model_tr(Config.armfsm.hosegrip.clearance)
        print("Target tr:",arm_planner.print_transform(trLArmTarget1))
        local arm_seq = {{'move',trLArmTarget1,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armovertool" end
      end
    end
  elseif stage=="armovertool" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        local trLArmTarget1 = get_model_tr({0,0,0})
        print("Target tr:",arm_planner.print_transform(trLArmTarget1))
        local arm_seq = {{'move',trLArmTarget1,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armtouchtool" end
      elseif hcm.get_state_proceed()==2 then 
        update_model()
        local trLArmTarget1 = get_model_tr(Config.armfsm.hosegrip.clearance)
        print("Target tr:",arm_planner.print_transform(trLArmTarget1))
        local arm_seq = {{'move',trLArmTarget1,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armovertool" end
      end
    end
  elseif stage=="armtouchtool" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        stage = "armgrabtool" 
      elseif hcm.get_state_proceed()==-1 then 
        local trLArmTarget1 = get_model_tr(Config.armfsm.hosegrip.clearance)
        print("Target tr:",arm_planner.print_transform(trLArmTarget1))
        local arm_seq = {{'move',trLArmTarget1,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armovertool" end        
      elseif hcm.get_state_proceed()==2 then 
        update_model()
        local trLArmTarget1 = get_model_tr({0,0,0})
        print("Target tr:",arm_planner.print_transform(trLArmTarget1))
        local arm_seq = {{'move',trLArmTarget1,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armtouchtool" end
      end
    end
  elseif stage=="armgrabtool" then
    gripL,doneL = util.approachTol(gripL,1,2,dt)  --open gripper
    Body.set_lgrip_percent(gripL*0.8)
    if doneL then
      if hcm.get_state_proceed()==1 then 
        local trLArmTarget1 = get_model_tr(Config.armfsm.hosegrip.clearance)
        local arm_seq = {{'move',trLArmTarget1,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armlifttool" end
      end
    end

  elseif stage=="armlifttool" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        local trLArmTarget1 = Config.armfsm.hosegrip.arminit[2]
        local trLArmTarget2 = Config.armfsm.hosegrip.arminit[1]
        local trLArmTarget3 = Config.armfsm.hosegrip.armuninit[1]
        local trLArmTarget4 = Config.armfsm.hosegrip.armuninit[1]
        local arm_seq = {
            {'move',trLArmTarget1,nil},
            {'move',trLArmTarget2,nil},
            {'wrist',trLArmTarget3,nil},
            {'move',trLArmTarget4,nil},
          }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armmoveback" end
      end
    end
  elseif stage=="armmoveback" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        print("qLArm:",arm_planner.print_jangle(qLArm))
        stage = "shoulderdown1"
      end
    end
  elseif stage=="shoulderdown1" then
    local qLArmNew,done = util.approachTol(qLArm,
      Config.armfsm.hosegrip.armunflip[1],
      Config.arm.slow_limit,dt)
    movearm.setArmJoints(qLArmNew,qRArm,dt)
    if done then stage="shoulderdown2" end
  elseif stage=="shoulderdown2" then
    local qLArmNew,done = util.approachTol(qLArm,
      Config.armfsm.hosegrip.armunflip[2],
      Config.arm.slow_limit,dt)
    movearm.setArmJoints(qLArmNew,qRArm,dt)
    if done then stage="shoulderdown3" end
  elseif stage=="shoulderdown3" then
    local qLArmNew,done = util.approachTol(qLArm,
      Config.armfsm.hosegrip.armunflip[3],
      Config.arm.slow_limit,dt)
    movearm.setArmJoints(qLArmNew,qRArm,dt)
    if done then stage="shoulderdown4" end
  elseif stage=="shoulderdown4" then
    local qLArmNew,done = util.approachTol(qLArm,
      Config.armfsm.hosegrip.armunflip[4],
      Config.arm.slow_limit,dt)
    movearm.setArmJoints(qLArmNew,qRArm,dt)    
    if done then stage="shoulderdown5" end
  elseif stage=="shoulderdown5" then
    local qLArmNew,done = util.approachTol(qLArm,
      Config.armfsm.hosegrip.armunflip[5],
      Config.arm.slow_limit,dt)
    movearm.setArmJoints(qLArmNew,qRArm,dt)    
    if done then stage="shoulderdown6" end
  elseif stage=="shoulderdown6" then
    local qLArmNew,done = util.approachTol(qLArm,
      Config.armfsm.hosegrip.armunflip[6],
      Config.arm.slow_limit,dt)
    movearm.setArmJoints(qLArmNew,qRArm,dt)    
    if done then stage="shoulderdown7" end
  elseif stage=="shoulderdown7" then
    local qLArmNew,done = util.approachTol(qLArm,
      Config.armfsm.hosegrip.armunflip[7],
      Config.arm.slow_limit,dt)
    movearm.setArmJoints(qLArmNew,qRArm,dt)    
    if done then stage="alldone" end

  elseif stage=="alldone" then --Move arm back to holding position
    print("SEQUENCE DONE")
    arm_planner:reset_torso_comp(qLArm0,qRArm0)  --reset torso compensation
    return"hold"      
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