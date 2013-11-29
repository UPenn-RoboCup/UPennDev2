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
local lhand_rpy0 = Config.armfsm.dooropen.lhand_rpy

local rollTarget = Config.armfsm.dooropenleft.rollTarget
local yawTargetInitial = Config.armfsm.dooropenleft.yawTargetInitial
local yawTarget1 = Config.armfsm.dooropenleft.yawTarget1
local yawTarget2 = Config.armfsm.dooropenleft.yawTarget2


local trLArm0, trRArm0, trLArm1, trRArm1, qLArm0, qRarm0
local gripL, gripR = 1,1
local stage

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_lhandoffset(Config.arm.handoffset.outerhook)
  mcm.set_arm_rhandoffset(Config.arm.handoffset.outerhook)
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  qLArm0 = qLArm
  qRArm0 = qRArm
  
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  
  
  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})  
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, Config.armfsm.dooropenleft.rhand_push[1])
  
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

  arm_planner:set_shoulder_yaw_target(nil,qRArm0[3])--unlock left shoulder

  hcm.set_door_model(Config.armfsm.dooropenleft.default_model)
  hcm.set_door_yaw(0)

  hcm.set_state_tstartactual(unix.time()) 
  hcm.set_state_tstartrobot(Body.get_time())

  
--  local wrist_seq = {{'wrist',trLArm1, nil}}
  local wrist_seq = {{'wrist',trLArm1, trRArm1}}
  if arm_planner:plan_arm_sequence(wrist_seq) then stage = "wristturn" end
end

local function update_model()
  
  local trLArmTarget = hcm.get_hands_left_tr_target()
  local trLArm = hcm.get_hands_left_tr()
  local door_model = hcm.get_door_model()
  door_model[1],door_model[2],door_model[3]=
    door_model[1] + trLArmTarget[1]-trLArm[1],
    door_model[2] + trLArmTarget[2]-trLArm[2],
    door_model[3] + trLArmTarget[3]-trLArm[3]
  hcm.set_door_model(door_model)

  print(string.format("Door model update: hinge %.3f %.3f %.3f",
    door_model[1],door_model[2],door_model[3]))
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
  local qLArm = cur_cond[1]
  local qRArm = cur_cond[2]
  local trLArm = Body.get_forward_larm(cur_cond[1])
  local trRArm = Body.get_forward_rarm(cur_cond[2])  
  
  door_yaw = 0

  if stage=="wristturn" then --Turn yaw angles first
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        local trArmTarget = movearm.getDoorHandlePosition(
          Config.armfsm.dooropen.handle_clearance0, 0, door_yaw,1)
        local arm_seq = {{'move',trArmTarget, nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "placehook"  end
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])
        local wrist_seq = {{'wrist',trLArm0, nil}}
        if arm_planner:plan_arm_sequence(wrist_seq) then stage = "armbacktoinitpos" end
      end
    end
  elseif stage=="placehook" then --Move the hook below the door knob
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then        
        local trArmTarget1 = movearm.getDoorHandlePosition(
          Config.armfsm.dooropen.handle_clearance1, 0, door_yaw, 1)
        local trArmTarget2 = movearm.getDoorHandlePosition(
          {0,0,0}, 0, door_yaw, 1)
        local arm_seq = {{'move',trArmTarget1,nil},{'move',trArmTarget2,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "hookknob"  end
      elseif hcm.get_state_proceed()==-1 then
        local arm_seq = {{'move',nil,trRArm1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "wristyawturn" end        
      elseif hcm.get_state_proceed()==2 then        
        update_model()
        local trArmTarget1 = movearm.getDoorHandlePosition(
          Config.armfsm.dooropen.handle_clearance0, 0, door_yaw, 1)
        local arm_seq = {{'move',trArmTarget1,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "placehook"  end
      end      
    end    
    hcm.set_state_proceed(0) --Stop here for a bit

  elseif stage=="hookknob" then --Move up the hook to make it touch the knob
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then --Open the door
        arm_planner:save_doorparam({{0,0,0},0*Body.DEG_TO_RAD,0,0})
        local dooropen_seq =         {
            {'doorleft',{0,0,0}, rollTarget,door_yaw},
            {'doorleft',{0,0,0}, rollTarget,yawTargetInitial},
            {'doorleft',{0,0,0},  0*Body.DEG_TO_RAD,yawTargetInitial}   
          }
        if arm_planner:plan_arm_sequence2(dooropen_seq) then stage = "opendoor"  end
      elseif hcm.get_state_proceed()==-1 then --Lower the hook
        local trArmTarget1 = movearm.getDoorHandlePosition(Config.armfsm.dooropen.handle_clearance1, 0, door_yaw,1)
        local trArmTarget2 = movearm.getDoorHandlePosition(Config.armfsm.dooropen.handle_clearance0, 0, door_yaw,1)
        local arm_seq = {{'move',trArmTarget1,nil},{'move',trArmTarget2,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "placehook"  
          else hcm.set_state_proceed(0) end

      elseif hcm.get_state_proceed()==2 then --adjust hook position
        update_model()
        local trArmTarget1 = movearm.getDoorHandlePosition({0,0,0}, 0, door_yaw,1)
        local arm_seq = {{'move',trArmTarget1},nil}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "hookknob"  end
      end
    end
    if hcm.get_state_proceed()==1 then hcm.set_state_proceed(0) end --stop here and wait
  elseif stage=="opendoor" then --Move the arm forward using IK now     
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        local dooropen_seq ={
          {'doorleft',Config.armfsm.dooropen.handle_clearance2, 0*Body.DEG_TO_RAD,yawTargetInitial},
          {'doorleft',Config.armfsm.dooropen.handle_clearance2, 0*Body.DEG_TO_RAD,0},
--        {'wrist',Config.armfsm.dooropenleft.lhand_push,nil}
        }
        if arm_planner:plan_arm_sequence2(dooropen_seq) then stage = "opendoor2"  end
      elseif hcm.get_state_proceed()==-1 then --Re-hook
          local doorclose_seq={ 
            {'doorleft',Config.armfsm.dooropen.handle_clearance1,0,yawTargetInitial},
            {'doorleft',Config.armfsm.dooropen.handle_clearance1,0*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD},   
            {'doorleft',{0,0,0}, 0,0},
          }
          if arm_planner:plan_arm_sequence2(doorclose_seq) then stage = "hookknob"  end
      end
    end
    hcm.set_state_proceed(0)
  elseif stage=="opendoor2" then --Move the arm forward using IK now     
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        print("trRArm:",arm_planner.print_transform(trRArm))        
        arm_planner:set_shoulder_yaw_target(qLArm0[3],nil)
        local arm_seq = {
            {'move',trLArm1,Config.armfsm.dooropenleft.rhand_push[2]},
            {'move',nil,Config.armfsm.dooropenleft.rhand_push[3]},
            {'wrist',nil,Config.armfsm.dooropenleft.rhand_push[4]},
            {'wrist',nil,Config.armfsm.dooropenleft.rhand_push[3]},
            {'move',nil,Config.armfsm.dooropenleft.rhand_push[3]},
            {'move',nil,Config.armfsm.dooropenleft.rhand_push[2]},
          }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "pushdoor"  
          else print("not possible") end
      end
    end

  elseif stage=="pushdoor" then --Move the arm forward using IK now         
    if arm_planner:play_arm_sequence(t) then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])
        local arm_seq = {
          {'move',nil,trRArm1},            
          {'wrist',trLArm0,trRArm0},            
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armbacktoinitpos" end
    end

  elseif stage=="armbacktoinitpos" then 
    if arm_planner:play_arm_sequence(t) then return "done" end
  end
  hcm.set_state_proceed(0)
end

function state.exit()    
  print(state._NAME..' Exit' )
end

return state
