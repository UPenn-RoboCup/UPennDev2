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

local rollTarget = -45*Body.DEG_TO_RAD
local yawTarget = 20*Body.DEG_TO_RAD
local yawTargetInitial = 8*Body.DEG_TO_RAD

local qLArmInit0,qRArmInit0,qLArmInit1,qRArmInit1

local trLArmCurrent, trRArmCurrent

local trLArmRelease = {0,0,0,   90*Body.DEG_TO_RAD,0,0}
local trRArmRelease = {0,0,0,   -90*Body.DEG_TO_RAD,65*Body.DEG_TO_RAD,0}
--local trRArmRelease = {0,0,0,   0*Body.DEG_TO_RAD,65*Body.DEG_TO_RAD,0}

local debugdata

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_handoffset(Config.arm.handoffset.outerhook)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  -- -5 and 5 degrees default

  arm_planner:reset_torso_comp(qLArm, qRArm)
  arm_planner:save_boundary_condition({qLArm, qRArm, qLArm, qRArm, {0,0}})  

--  qLArm0 = qLArm
  qLArm0 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})
  qRArm0 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

  arm_planner:set_shoulder_yaw_target(qLArm[3],nil) --Lock left shoulder yaw

  local wrist_seq = { armseq={ {trLArm0,trRArm0}} }
  if arm_planner:plan_wrist_sequence(wrist_seq) then
    stage = "wristyawturn"    
  end


--  arm_planner:reset_torso_comp(qLArm1, qRArm1)
--  arm_planner:save_boundary_condition({qLArm1, qRArm1, qLArm1, qRArm1, {0,0}})


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
  debugdata=''
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
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        local trRArmTarget1 = movearm.getDoorHandlePosition(handle_clearance, 0, door_yaw, rhand_rpy0)
        local arm_seq = {armseq={{trLArm0, trRArmTarget1}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "placehook"  end
      end
    end  
  elseif stage=="placehook" then --Move the arm forward using IK now       
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then --Put the hook on
        local trRArmTarget2 = movearm.getDoorHandlePosition({0,0,0}, 0, door_yaw, rhand_rpy0)
        local arm_seq = {armseq={{trLArm0, trRArmTarget2}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "hookknob"  end
      elseif hcm.get_state_proceed()==2 then --adjust hook position
        local trRArmCurrent = hcm.get_hands_right_tr()
        local trRArmTarget = hcm.get_hands_right_tr_target()
        local door_model = hcm.get_door_model()
        door_model[1],door_model[2],door_model[3] = 
          trRArmTarget[1] - trRArmCurrent[1] + door_model[1],
          trRArmTarget[2] - trRArmCurrent[2] + door_model[2],
          trRArmTarget[3] - trRArmCurrent[3] + door_model[3]
        hcm.set_door_model(door_model)

        local trRArmTarget1 = movearm.getDoorHandlePosition(handle_clearance, 0, door_yaw, rhand_rpy0)
        local arm_seq = {armseq={{trLArm0, trRArmTarget1}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "placehook"  end

      end
    end
  elseif stage=="hookknob" then --Move the arm forward using IK now       
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then --Open the door
        arm_planner:save_doorparam({{0,0,0},0*Body.DEG_TO_RAD,0,0})
        local dooropen_seq =         
          { {{0,0,0}, rollTarget,door_yaw},
            {{0,0,0}, rollTarget,yawTargetInitial},
            {{0,0,0},  0*Body.DEG_TO_RAD,yawTargetInitial}   }
        if arm_planner:plan_open_door_sequence(dooropen_seq) then stage = "opendoor"  end
      elseif hcm.get_state_proceed()==-1 then --Lower the hook
        local trRArmTarget2 = movearm.getDoorHandlePosition(handle_clearance, 0, door_yaw, rhand_rpy0)
        local arm_seq = {armseq={{trLArm0, trRArmTarget2}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "placehook"  end
      end
    end
  elseif stage=="opendoor" then --Move the arm forward using IK now     
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        local dooropen_seq = {          
          {{0,0,0},  0*Body.DEG_TO_RAD,yawTarget}--,-30*Body.DEG_TO_RAD},          
        }
        if arm_planner:plan_open_door_sequence(dooropen_seq) then stage = "opendoor2"  end  
      elseif hcm.get_state_proceed()==-1 then
          local doorclose_seq={ 
            {{0,0,-0.10},  0,yawTargetInitial},
            {{0,0,-0.10},  0*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD},   
            {{0,0,0}, 0,0},
          }
          if arm_planner:plan_open_door_sequence(doorclose_seq) then stage = "hookknob"  end
      end
    end
  elseif stage=="opendoor2" then 
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        local trRArmTarget1 = movearm.getDoorHandlePosition(
            {0,0,-0.10}, 0, yawTarget, rhand_rpy0)      
        local arm_seq = {armseq={{trLArm0, trRArmTarget1}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "hookrelease"  end
      end
    end
  elseif stage=="hookrelease" then 
    if arm_planner:play_arm_sequence(t) then 
      local wrist_seq = {  armseq={{trLArm0, trRArmRelease}}  }
      if arm_planner:plan_wrist_sequence(wrist_seq) then   
        stage = "wristreset"
      end
    end
  elseif stage=="wristreset" then
    if arm_planner:play_arm_sequence(t) then 
      local trRArmTarget = Body.get_forward_rarm(qRArm)
      print(unpack(trRArmTarget))
--      arm_planner:set_shoulder_yaw_target(-5*Body.DEG_TO_RAD, 5*Body.DET_TO_RAD) 
      arm_planner:set_shoulder_yaw_target(-5*Body.DEG_TO_RAD, 1*Body.DEG_TO_RAD) 
      trRArmTarget[2] = -0.25;      

      local arm_seq = {
        armseq={
          {trLArm0, trRArmTarget},
          {trLArm0, trRArmTarget+ vector.new({0.30,0,0.05, 0,0,0})},
--          {trLArm0, trRArmTarget + vector.new({0.10,0,0.10, 0,0,0})}
        } 
      }
      if arm_planner:plan_arm_sequence(arm_seq) then stage = "armmoveback" end
    end
  elseif stage=="armmoveback" then
    if arm_planner:play_arm_sequence(t) then 
      stage="armpushdoor"
    end
  end
  debugdata = debugdata..string.format("%.3f,  %.3f,%.3f,%.3f\n",
    t-t_entry,
    qRArm[5]*Body.RAD_TO_DEG,
    qRArm[6]*Body.RAD_TO_DEG,
    qRArm[7]*Body.RAD_TO_DEG
    )
  hcm.set_state_proceed(0)
end

local function flush_debugdata()
  local savefile = string.format("Log/debugdata_%s",os.date());
  local debugfile=assert(io.open(savefile,"w")); 
  debugfile:write(debugdata);
  debugfile:flush();
  debugfile:close();  
end

function state.exit()  
  flush_debugdata()
  print(state._NAME..' Exit' )
end



return state
