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
local rhand_rpy0 = {-90*Body.DEG_TO_RAD,-5*Body.DEG_TO_RAD,0}
local trLArm1, trRArm1
local stage

local qLArmInit0,qRArmInit0,qLArmInit1,qRArmInit1

local trLArmCurrent, trRArmCurrent

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_handoffset(Config.arm.handoffset.outerhook)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  --First init jangles (wrist roll straight)
  qLArm0 = qLArm
  qRArm0 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  qRArm0[6] = 0,0

  --Second init angles (bent wrist roll)
  qLArm1 = qLArm
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})

  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

--The door we have: hinge height 93.98
--door r: 86.36

  local hinge_pos = vector.new({0.59,-1.15,0.01})
  --This considers the hook offset

  local door_r = 0.86
  local grip_offset_x = -0.05
  local knob_offset_y = 0.08
--  local knob_offset_y = 0.15 --for testing
  
  hcm.set_door_model({
    hinge_pos[1],hinge_pos[2],hinge_pos[3],
    door_r,
    grip_offset_x,
    knob_offset_y})
  hcm.set_door_yaw(0)

  arm_planner:reset_torso_comp(qLArm1, qRArm1)
  arm_planner:save_boundary_condition({qLArm1, qRArm1, qLArm1, qRArm1, {0,0}})
  stage = "wristyawturn";  
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
  
  local door_yaw = hcm.get_door_yaw()
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()


  if stage=="wristyawturn" then --Turn yaw angles first
    if movearm.setArmJoints(qLArm0,qRArm0,dt, Config.arm.joint_init_limit) ==1 then 
      stage = "wristrollturn"
    end
  elseif stage=="wristrollturn" then       
    if movearm.setArmJoints(qLArm1,qRArm1,dt, Config.arm.joint_init_limit) ==1 then
      if hcm.get_state_proceed()==1 then
        local trRArmTarget1 = movearm.getDoorHandlePosition(handle_clearance, 0, door_yaw, rhand_rpy0)
        local arm_seq = {armseq={{trLArm1, trRArmTarget1}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "placehook"  end
      end
    end    
  elseif stage=="placehook" then --Move the arm forward using IK now       
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then --Put the hook on
        local trRArmTarget2 = movearm.getDoorHandlePosition({0,0,0}, 0, door_yaw, rhand_rpy0)
        local arm_seq = {armseq={{trLArm1, trRArmTarget2}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "hookknob"  end
      end
    end
  elseif stage=="hookknob" then --Move the arm forward using IK now       
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then --Open the door
        arm_planner:save_doorparam({{0,0,0},0*Body.DEG_TO_RAD,0,0})
        local dooropen_seq =         
          { {{0,0,0},-30*Body.DEG_TO_RAD,door_yaw},
            {{0,0,0},-30*Body.DEG_TO_RAD,6*Body.DEG_TO_RAD},          
            {{0,0,0},  0*Body.DEG_TO_RAD,6*Body.DEG_TO_RAD}   }
        if arm_planner:plan_open_door_sequence(dooropen_seq) then stage = "opendoor"  end
      elseif hcm.get_state_proceed()==-1 then --Lower the hook
        local trRArmTarget2 = movearm.getDoorHandlePosition(handle_clearance, 0, door_yaw, rhand_rpy0)
        local arm_seq = {armseq={{trLArm1, trRArmTarget2}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "placehook"  end
      end
    end
  elseif stage=="opendoor" then --Move the arm forward using IK now     
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        local dooropen_seq = {          
          {{0,0,0},  0*Body.DEG_TO_RAD,30*Body.DEG_TO_RAD}--,-30*Body.DEG_TO_RAD},          
        }
        if arm_planner:plan_open_door_sequence(dooropen_seq) then stage = "opendoor2"  end  
      end
    end
  elseif stage=="opendoor2" then --Move the arm forward using IK now     
    if arm_planner:play_arm_sequence(t) then 
      local trRArmTarget1 = movearm.getDoorHandlePosition(
          {0,0,-0.10}, 0, 30*Body.DEG_TO_RAD, rhand_rpy0)      
      local arm_seq = {armseq={{trLArm1, trRArmTarget1}} }
      if arm_planner:plan_arm_sequence(arm_seq) then stage = "hookrelease"  end
    end
  elseif stage=="hookrelease" then --Move the arm forward using IK now     
    if arm_planner:play_arm_sequence(t) then 
     
    end
  end
  hcm.set_state_proceed(0)
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
