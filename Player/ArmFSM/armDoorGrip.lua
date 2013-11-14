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
local trLArm0, trRArm0, trLArm1, trRArm1
local stage

local rollTarget = -45*Body.DEG_TO_RAD
local yawTargetInitial = 8*Body.DEG_TO_RAD

local yawTarget = 40*Body.DEG_TO_RAD
local yawTarget = 30*Body.DEG_TO_RAD


local qLArmInit0,qRArmInit0,qLArmInit1,qRArmInit1

local trLArmCurrent, trRArmCurrent

local trLArmRelease = {0,0,0,   90*Body.DEG_TO_RAD,0,0}
local trRArmRelease = {0,0,0,   -90*Body.DEG_TO_RAD,45*Body.DEG_TO_RAD,0}

local trRArmForward = {0,0,0,   -90*Body.DEG_TO_RAD,25*Body.DEG_TO_RAD,0}

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
  

  arm_planner:reset_torso_comp(qLArm, qRArm)
  arm_planner:save_boundary_condition({qLArm, qRArm, qLArm, qRArm, {0,0}})  

  qLArm0 = qLArm
  qRArm0 = qRArm
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

--  qLArm0 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})
  qLArm1 = qLArm
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

  -- Default shoulder yaw angle: (-5,5)
  arm_planner:set_shoulder_yaw_target(qLArm0[3],nil) --Lock left shoulder yaw
  local wrist_seq = { armseq={ {trLArm1,trRArm1}} }
  if arm_planner:plan_wrist_sequence(wrist_seq) then
    stage = "wristyawturn"    
  end


--The door we have: hinge height 93.98
--door r: 86.36

  local hinge_pos = vector.new({0.59,-1.15,0.01})
  --This considers the hook offset

  local door_r = 0.86
  local grip_offset_x = -0.05
  local knob_offset_y = 0.08
  
  hcm.set_door_model({
    hinge_pos[1],hinge_pos[2],hinge_pos[3],
    door_r,
    grip_offset_x,
    knob_offset_y})
  hcm.set_door_yaw(0)
  debugdata=''
end

local function update_model()
  local trRArmCurrent = hcm.get_hands_right_tr()
  local trRArmTarget = hcm.get_hands_right_tr_target()
  local door_model = hcm.get_door_model()
  door_model[1],door_model[2],door_model[3] = 
    trRArmTarget[1] - trRArmCurrent[1] + door_model[1],
    trRArmTarget[2] - trRArmCurrent[2] + door_model[2],
    trRArmTarget[3] - trRArmCurrent[3] + door_model[3]
  hcm.set_door_model(door_model)
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

  if stage=="wristyawturn" then --Turn wrist angles without moving arms
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then        
        arm_planner:set_shoulder_yaw_target(qLArm0[3],nil) --Lock left shoulder yaw
        local trRArmTarget1 = movearm.getDoorHandlePosition(handle_clearance, 0, door_yaw, rhand_rpy0)
        local arm_seq = {armseq={{trLArm1, trRArmTarget1}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "placehook"  end
      elseif hcm.get_state_proceed()==-1 then
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])
        local wrist_seq = { armseq={ {trLArm0,trRArm0}} }
        if arm_planner:plan_wrist_sequence(wrist_seq) then stage = "armbacktoinitpos" end   
      end
    end  
  elseif stage=="placehook" then --Move the hook below the door knob
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then --Put the hook on
        local trRArmTarget2 = movearm.getDoorHandlePosition({0,0,0}, 0, door_yaw, rhand_rpy0)
        local arm_seq = {armseq={{trLArm1, trRArmTarget2}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "hookknob"  end

      elseif hcm.get_state_proceed()==-1 then
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])--Lock both shoulder yaw
        local arm_seq = {armseq={{trLArm1, trRArm1}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "wristyawturn" end        

      elseif hcm.get_state_proceed()==2 then --adjust hook position
        update_model()
        local trRArmTarget1 = movearm.getDoorHandlePosition(handle_clearance, 0, door_yaw, rhand_rpy0)
        local arm_seq = {armseq={{trLArm1, trRArmTarget1}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "placehook"  end

      end
    end
  elseif stage=="hookknob" then --Move up the hook to make it touch the knob
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
        local arm_seq = {armseq={{trLArm1, trRArmTarget2}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "placehook"  end

      elseif hcm.get_state_proceed()==2 then --adjust hook position
        update_model()
        local trRArmTarget1 = movearm.getDoorHandlePosition({0,0,0}, 0, door_yaw, rhand_rpy0)
        local arm_seq = {armseq={{trLArm1, trRArmTarget1}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "hookknob"  end

      end
    end
  elseif stage=="opendoor" then --Move the arm forward using IK now     
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        local dooropen_seq = { {{0,0,0},  0*Body.DEG_TO_RAD,yawTarget} }
        if arm_planner:plan_open_door_sequence(dooropen_seq) then stage = "opendoor2"  end  

      elseif hcm.get_state_proceed()==-1 then --Re-hook
          local doorclose_seq={ 
            {handle_clearance,  0,yawTargetInitial},
            {handle_clearance,  0*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD},   
            {{0,0,0}, 0,0},
          }
          if arm_planner:plan_open_door_sequence(doorclose_seq) then stage = "hookknob"  end

      end
    end
  elseif stage=="opendoor2" then 
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        local doorparam = arm_planner.init_doorparam
        
        --local trRArmTarget1 = movearm.getDoorHandlePosition(handle_clearance, 0, yawTarget, rhand_rpy0)      
        local trRArmTarget1 = movearm.getDoorHandlePosition(handle_clearance, 0, doorparam[3], rhand_rpy0)
        local trRArmTarget2 = movearm.getDoorHandlePosition(
          vector.new(handle_clearance)+vector.new({-0.05,0,0}), 0, doorparam[3], rhand_rpy0)      

        local arm_seq = {armseq={{trLArm1, trRArmTarget1},{trLArm1, trRArmTarget2}} }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "hookrelease"  end
      elseif hcm.get_state_proceed()==-1 then --Fully close the door
          local doorclose_seq={ 
            {{0,0,0},  0,0},            
          }
          if arm_planner:plan_open_door_sequence(doorclose_seq) then stage = "hookknob"  end
      end
    end
  elseif stage=="hookrelease" then 
    arm_planner:set_shoulder_yaw_target(-5*Body.DEG_TO_RAD, 5*Body.DEG_TO_RAD)           
    if arm_planner:play_arm_sequence(t) then 
      local wrist_seq = {  armseq={{trLArm1, trRArmRelease}}  }
      if arm_planner:plan_wrist_sequence(wrist_seq) then stage = "hookrollback"  end
    end
  elseif stage=="hookrollback" then
    if arm_planner:play_arm_sequence(t) then 

      local trRArmTarget = Body.get_forward_rarm(qRArm)
      local trRArmTarget1 = Body.get_forward_rarm(qRArm)

      trRArmTarget1[2] = -0.25;
--      arm_planner:set_shoulder_yaw_target(-5*Body.DEG_TO_RAD, 5*Body.DEG_TO_RAD) 
      arm_planner:set_shoulder_yaw_target(-5*Body.DEG_TO_RAD, nil) 
--      arm_planner:set_shoulder_yaw_target(-5*Body.DEG_TO_RAD, 15*Body.DEG_TO_RAD)       

      local arm_seq = {
        armseq={ 
                 {trLArm1, trRArmTarget+ vector.new({0.05,0,0, 0,0,0})},
                 {trLArm1, trRArmTarget1+ vector.new({0.05,0,0, 0,0,0})}
              } 
          }
      if arm_planner:plan_arm_sequence(arm_seq) then stage = "hookforward" end      
    end
  elseif stage=="hookforward" then
    if arm_planner:play_arm_sequence(t) then 
--      local wrist_seq = {  armseq={{trLArm1, trRArmForward}}  }
      local wrist_seq = {  armseq={{trLArm1, {0,0,0,unpack(rhand_rpy0)}}  }}
      if arm_planner:plan_wrist_sequence(wrist_seq) then stage = "armmoveside"  end      
    end    
  elseif stage=="armmoveside" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        arm_planner:set_shoulder_yaw_target(-5*Body.DEG_TO_RAD, qRArm[3]) 
        local trRArmTarget = Body.get_forward_rarm(qRArm)
        local arm_seq = {
          armseq={ 
            {trLArm1, trRArmTarget+ vector.new({0.15,0,0,  0,0,0})},
          } 
        }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "hookside" end
      end
    end
  elseif stage=="hookside" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        arm_planner:set_shoulder_yaw_target(-5*Body.DEG_TO_RAD, nil) 
        local arm_seq = {
          armseq={ 
            {trLArm1, trRArm0},
          } 
        }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "armbacktoinitpos" end
      end    
    end
  elseif stage=="armbacktoinitpos" then
    if arm_planner:play_arm_sequence(t) then return "done" end
  end
  
  hcm.set_state_proceed(0)
end

local function log_debugdata(qRArm)
  debugdata = debugdata..string.format("%.3f,  %.3f,%.3f,%.3f\n",
    t-t_entry,
    qRArm[5]*Body.RAD_TO_DEG,
    qRArm[6]*Body.RAD_TO_DEG,
    qRArm[7]*Body.RAD_TO_DEG
    )
end

local function flush_debugdata()
  local savefile = string.format("Log/debugdata_%s",os.date());
  local debugfile=assert(io.open(savefile,"w")); 
  debugfile:write(debugdata);
  debugfile:flush();
  debugfile:close();  
end

function state.exit()  
--  flush_debugdata()
  print(state._NAME..' Exit' )
end



return state
