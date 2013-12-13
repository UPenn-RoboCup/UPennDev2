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
local rollTarget, yawTarget = 0,0


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
  hcm.set_state_proceed(1) --stop here and wait
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
    hcm.set_state_proceed(0)
end



local function update_override()
  local overrideTarget = hcm.get_state_override_target()
  local override = hcm.get_state_override()
  local door_model = hcm.get_door_model()

  door_model={
    door_model[1] + overrideTarget[1]-override[1],
    door_model[2] + overrideTarget[2]-override[2],
    door_model[3] + overrideTarget[3]-override[3],
    door_model[4], --Door width
    door_model[5], -- Knob x offset
    door_model[6], -- Knob y offset from knob axle
    door_model[7] + overrideTarget[4]-override[4], --Target knob roll
    door_model[8] + overrideTarget[5]-override[5]  --Target door yaw
    }

  --Knob roll: 0 to -90
  door_model[7] = math.max(-90*Body.DEG_TO_RAD,math.min(0,door_model[7]))

  --Door yaw : plus to pull, minus to push
  door_model[8] = math.max(-30*Body.DEG_TO_RAD,math.min(30*Body.DEG_TO_RAD,door_model[8]))  

  hcm.set_door_model(door_model)
  hcm.set_state_proceed(0)
  print( util.color('Door model:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f %.1f",
      door_model[1],door_model[2],door_model[3],
      door_model[7]*180/math.pi,
      door_model[8]*180/math.pi
         ))

  rollTarget = door_model[7]
  yawTarget = door_model[8]
end

local function revert_override()
  local overrideTarget = hcm.get_state_override_target()
  local override = hcm.get_state_override()
  local door_model = hcm.get_door_model()

  door_model={
    door_model[1] - (overrideTarget[1]-override[1]),
    door_model[2] - (overrideTarget[2]-override[2]),
    door_model[3] - (overrideTarget[3]-override[3]),
    door_model[4], --Door width
    door_model[5], -- Knob x offset
    door_model[6], -- Knob y offset from knob axle
    door_model[7] - (overrideTarget[4]-override[4]), --Target knob roll
    door_model[8] - (overrideTarget[5]-override[5])  --Target door yaw
    }

  --Knob roll: 0 to -90
  door_model[7] = math.max(-90*Body.DEG_TO_RAD,math.min(0,door_model[7]))

  --Door yaw : plus to pull, minus to push
  door_model[8] = math.max(-30*Body.DEG_TO_RAD,math.min(30*Body.DEG_TO_RAD,door_model[8]))  

  hcm.set_door_model(door_model)
  hcm.set_state_proceed(0)
  print( util.color('Door model:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f %.1f",
      door_model[1],door_model[2],door_model[3],
      door_model[7]*180/math.pi,
      door_model[8]*180/math.pi
         ))

  rollTarget = door_model[7]
  yawTarget = door_model[8]
end

local function confirm_override()
  local override = hcm.get_state_override()
  hcm.set_state_override_target(override)
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
        local trArmTarget = movearm.getDoorHandlePosition({0,0,0}, 0, door_yaw, 1)
        local arm_seq = {{'move',trArmTarget, nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "opendoor"  end
        hcm.set_state_proceed(0) --stop here and wait
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])
        local wrist_seq = {{'wrist',trLArm0, trRArm0}}
        if arm_planner:plan_arm_sequence(wrist_seq) then stage = "armbacktoinitpos" end
      end
    end


  elseif stage=="opendoor" then 
    --Manually open the door
    --Control: xyz, knob roll, door yaw

    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then --Unhook left arm
        arm_planner:save_doorparam({{0,0,0},rollTarget,yawTarget,0})
        yawTarget1 = yawTarget
        local dooropen_seq ={ 
          {'doorleft',{0,0,0},  0*Body.DEG_TO_RAD,yawTarget1},
--          {'doorleft',Config.armfsm.dooropen.handle_clearance2,0,yawTarget1}
        }
        if arm_planner:plan_arm_sequence2(dooropen_seq) then stage = "hookrelease"  end
      elseif hcm.get_state_proceed()==-1 then --Lower the hook
        local trArmTarget1 = movearm.getDoorHandlePosition(Config.armfsm.dooropen.handle_clearance1, 0, door_yaw,1)
        local arm_seq = {{'move',trtrArmTarget1,nil},{'move',trLArm1,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "wristturn"  
        else hcm.set_state_proceed(0) end
      elseif hcm.get_state_proceed()==2 then --adjust hook position
        update_model()
        local trArmTarget1 = movearm.getDoorHandlePosition({0,0,0}, 0, door_yaw,1)
        local arm_seq = {{'move',trArmTarget1},nil}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "hookknob"  end
      elseif hcm.get_state_proceed()==3 then --adjust hook position
        update_override()
        local trArmTarget1 = movearm.getDoorHandlePosition({0,0,0}, rollTarget, yawTarget,1)
        local arm_seq = {{'move',trArmTarget1,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then 
          stage = "opendoor"  
          confirm_override()
        else
          revert_override()
        end
      end
    end    

  elseif stage=="hookrelease" then --Move the arm forward using IK now     
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        print("trRArm:",arm_planner.print_transform(trRArm))        
        arm_planner:set_shoulder_yaw_target(qLArm0[3],nil)
        local arm_seq = {
            {'move',trLArm1,Config.armfsm.dooropenleft.rhand_push[2]},
            {'move',nil,Config.armfsm.dooropenleft.rhand_push[3]},
          }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "pushdoor" end
        hcm.set_state_proceed(0)
      end
    end
  elseif stage=="pushdoor" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        local arm_seq = {
          {'wrist',nil,Config.armfsm.dooropenleft.rhand_push[4]},
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "swingdoor" end
        hcm.set_state_proceed(0)
      elseif hcm.get_state_proceed()==3 then --manual movement
        local trRArmCurrent = hcm.get_hands_right_tr()
        local overrideTarget = hcm.get_state_override_target()
        local override = hcm.get_state_override()
        local trRArmTarget = {
          trRArmCurrent[1]+overrideTarget[1]-override[1],
          trRArmCurrent[2]+overrideTarget[2]-override[2],
          trRArmCurrent[3]+overrideTarget[3]-override[3],
          trRArmCurrent[4],
          trRArmCurrent[5],
          trRArmCurrent[6]
        }
        local arm_seq = {{'move',nil, trRArmTarget}}
        if arm_planner:plan_arm_sequence2(arm_seq) then 
          stage = "pushdoor" 
          confirm_override()
        else
          revert_override()
        end
        hcm.set_state_proceed(0)
      end
    end
  elseif stage=="swingdoor" then --Move the arm forward using IK now         
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        local arm_seq = {
          {'wrist',nil,Config.armfsm.dooropenleft.rhand_push[3]},
          {'move',nil,Config.armfsm.dooropenleft.rhand_push[3]},
          {'move',nil,Config.armfsm.dooropenleft.rhand_push[2]},
          {'move',nil,trRArm1},            
          {'wrist',trLArm0,trRArm0},            
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armbacktoinitpos" end
      elseif hcm.get_state_proceed()==3 then
        local trRArmCurrent = hcm.get_hands_right_tr()
        local overrideTarget = hcm.get_state_override_target()
        local override = hcm.get_state_override()
        local trRArmTarget = {
          trRArmCurrent[1]+overrideTarget[1]-override[1],
          trRArmCurrent[2]+overrideTarget[2]-override[2],
          trRArmCurrent[3]+overrideTarget[3]-override[3],
          trRArmCurrent[4],
          trRArmCurrent[5],
          trRArmCurrent[6]
        }
        local arm_seq = {{'move',nil, trRArmTarget}}
        if arm_planner:plan_arm_sequence2(arm_seq) then 
          stage = "swingdoor" 
          confirm_override()
        else
          revert_override()
        end

        hcm.set_state_proceed(0)
      end
    end
  elseif stage=="armbacktoinitpos" then 
    if arm_planner:play_arm_sequence(t) then 
          arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])    
      return "done" 
    end

  end

end

function state.exit()    
  print(state._NAME..' Exit' )
end

return state