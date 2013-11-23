local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()

--Door opening state using HOOK

local rhand_rpy0 = Config.armfsm.dooropen.rhand_rpy
local trRArmRelease = {0,0,0,unpack(Config.armfsm.dooropen.rhand_rpy_release)}
local trRArmRelease2 = {0,0,0,unpack(Config.armfsm.dooropen.rhand_rpy_release2)}
local trRArmRelease3 = {0,0,0,unpack(Config.armfsm.dooropen.rhand_rpy_release3)}


local trRArmForward = {0,0,0,unpack(Config.armfsm.dooropen.rhand_rpy_forward)}
local trRArmSidePush = {0,0,0,unpack(Config.armfsm.dooropen.rhand_rpy_sidepush)}

local handle_clearance = Config.armfsm.dooropen.handle_clearance
local rollTarget = Config.armfsm.dooropen.rollTarget
local yawTargetInitial = Config.armfsm.dooropen.yawTargetInitial
local yawTarget = Config.armfsm.dooropen.yawTarget


local trLArm0, trRArm0, trLArm1, trRArm1
local stage


local debugdata

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_rhandoffset(Config.arm.handoffset.outerhook)
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  qLArm0 = qLArm
  qRArm0 = qRArm
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

  qLArm1 = qLArm
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

  -- Default shoulder yaw angle: (-5,5)
  arm_planner:set_shoulder_yaw_target(qLArm0[3],nil) --Lock left shoulder yaw
  local wrist_seq = {{'wrist',nil,trRArm1}}
  if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "wristyawturn"end
 
  hcm.set_door_model(Config.armfsm.dooropen.default_model)
  hcm.set_door_yaw(0)
  debugdata=''

  hcm.set_state_tstartactual(unix.time()) 
  hcm.set_state_tstartrobot(Body.get_time())
end

local function update_model()
  local trRArmCurrent = hcm.get_hands_right_tr()
  local trRArmTarget = hcm.get_hands_right_tr_target()
  local door_model = hcm.get_door_model()
  door_model[1],door_model[2],door_model[3] = 
    trRArmTarget[1] - trRArmCurrent[1] + door_model[1],
    trRArmTarget[2] - trRArmCurrent[2] + door_model[2],
    trRArmTarget[3] - trRArmCurrent[3] + door_model[3]

  print(string.format("Door model update: hinge %.3f %.3f %.3f",
    door_model[1],door_model[2],door_model[3]))

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
  local cur_cond = arm_planner:load_boundary_condition()
  local trLArm = Body.get_forward_larm(cur_cond[1])
  local trRArm = Body.get_forward_rarm(cur_cond[2])  


  if stage=="wristyawturn" then --Turn wrist angles without moving arms
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then        
        arm_planner:set_shoulder_yaw_target(qLArm0[3],nil) --Lock left shoulder yaw
        local trRArmTarget1 = movearm.getDoorHandlePosition(handle_clearance, 0, door_yaw, rhand_rpy0)

        print("trRArm:",arm_planner.print_transform(trRArmTarget1))

        local arm_seq = {{'move',nil, trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "placehook"  end
      elseif hcm.get_state_proceed()==-1 then
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])
        local wrist_seq = {{'wrist',nil,trRArm0}}
        if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "armbacktoinitpos" end   
      end
    end  
  elseif stage=="placehook" then --Move the hook below the door knob
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then --Put the hook on
        local trRArmTarget2 = movearm.getDoorHandlePosition({0,0,0}, 0, door_yaw, rhand_rpy0)
        local arm_seq = {{'move',nil, trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "hookknob"  end

      elseif hcm.get_state_proceed()==-1 then
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])--Lock both shoulder yaw
        local arm_seq = {{'move',nil,trRArm1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "wristyawturn" end        

      elseif hcm.get_state_proceed()==2 then --adjust hook position
        update_model()
        local trRArmTarget1 = movearm.getDoorHandlePosition(handle_clearance, 0, door_yaw, rhand_rpy0)
        local arm_seq = {{'move',nil,trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "placehook"  end

      end
    end
  elseif stage=="hookknob" then --Move up the hook to make it touch the knob
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then --Open the door
        arm_planner:save_doorparam({{0,0,0},0*Body.DEG_TO_RAD,0,0})
        local dooropen_seq =         {
            {'door',{0,0,0}, rollTarget,door_yaw},
            {'door',{0,0,0}, rollTarget,yawTargetInitial},
            {'door',{0,0,0},  0*Body.DEG_TO_RAD,yawTargetInitial}   
          }
        if arm_planner:plan_arm_sequence2(dooropen_seq) then stage = "opendoor"  end

      elseif hcm.get_state_proceed()==-1 then --Lower the hook
        local trRArmTarget2 = movearm.getDoorHandlePosition(handle_clearance, 0, door_yaw, rhand_rpy0)
        local arm_seq = {{'move',nil,trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "placehook"  end

      elseif hcm.get_state_proceed()==2 then --adjust hook position
        update_model()
        local trRArmTarget1 = movearm.getDoorHandlePosition({0,0,0}, 0, door_yaw, rhand_rpy0)
        local arm_seq = {{'move',nil,trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "hookknob"  end

      end
    end
  elseif stage=="opendoor" then --Move the arm forward using IK now     
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then

        local dooropen_seq ={{'door',{0,0,0},  0*Body.DEG_TO_RAD,yawTarget}}
        if arm_planner:plan_arm_sequence2(dooropen_seq) then stage = "opendoor2"  end
      elseif hcm.get_state_proceed()==-1 then --Re-hook
          local doorclose_seq={ 
            {'door',handle_clearance,  0,yawTargetInitial},
            {'door',handle_clearance,  0*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD},   
            {'door',{0,0,0}, 0,0},
          }
          if arm_planner:plan_arm_sequence2(doorclose_seq) then stage = "hookknob"  end
      end
    end
  elseif stage=="opendoor2" then 
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        local doorparam = arm_planner.init_doorparam
        
        --local trRArmTarget1 = movearm.getDoorHandlePosition(handle_clearance, 0, yawTarget, rhand_rpy0)      
        local trRArmTarget1 = movearm.getDoorHandlePosition(handle_clearance, 0, doorparam[3], rhand_rpy0)
        local trRArmTarget2 = movearm.getDoorHandlePosition(
          vector.new(handle_clearance)+vector.new({-0.03,0,0}), 0, doorparam[3], rhand_rpy0)      

        local arm_seq = {{'move',nil,trRArmTarget1},{'move',nil,trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "hookrelease"  end
      elseif hcm.get_state_proceed()==-1 then --Fully close the door
          local doorclose_seq={{'door',{0,0,0},  0,0}}
          if arm_planner:plan_arm_sequence2(doorclose_seq) then stage = "hookknob"  end
      end
    end
  elseif stage=="hookrelease" then     
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then        
        local trRArmRelease1 = {0,0,0,trRArmRelease[4],trRArmRelease[5],trRArm[6]}
        local wrist_seq = {{'wrist',nil, trRArmRelease1},
                          {'wrist',nil, trRArmRelease},
                          {'wrist',nil, trRArmRelease2},
                          {'wrist',nil, trRArmRelease3},
                          }
        if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "hookrollback"  end
      end
    end
  elseif stage=="hookrollback" then
    if arm_planner:play_arm_sequence(t) then       
      if hcm.get_state_proceed()==1 then
      --move arm inside
        print("Current trRArm:",arm_planner.print_transform(trRArm))
        local trRArmTarget1 = vector.new(trRArm)+ vector.new({0,0.10,-0.10,  0,0,0})
        local trRArmTarget2 = {trRArm[1], -0.30, trRArm[3]-0.10,trRArm[4],trRArm[5],trRArm[6]}
        local trRArmTarget3 = {0.05, -0.30, -0.45,trRArm[4],trRArm[5],trRArm[6]}
        local arm_seq = {{'move',nil,trRArmTarget1},{'move',nil,trRArmTarget2},{'move',nil,trRArmTarget3}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "hookforward" end      
      end
    end
  elseif stage=="hookforward" then    
    if arm_planner:play_arm_sequence(t) then      
      if hcm.get_state_proceed()==1 then
        print("hookforward")
        local wrist_seq = {{'wrist',nil, trRArm0}}
--        local wrist_seq = {{'move',nil, trRArm0}}
--        if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "armmovefront"  end
        if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "armbacktoinitpos1"  end
      end
    end    













  elseif stage=="armmovefront" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        print("Current trRArm:",arm_planner.print_transform(trRArm))
        --local trRArmTarget = {0.13,-0.20,0.09, trRArmForward[4],trRArmForward[5],trRArmForward[6]} --current pos
        arm_planner:set_shoulder_yaw_target(-5*Body.DEG_TO_RAD, qRArm0[3]) 
        --local trRArmTarget = {0.26,-0.20,-0.09, trRArmForward[4],trRArmForward[5],trRArmForward[6]}
        local trRArmTarget1 = {0.30,-0.15,-0.09, trRArmForward[4],trRArmForward[5],trRArmForward[6]} 
        local trRArmTarget2 = {0.30,-0.15,-0.09, trRArmForward[4],trRArmForward[5],0} 
        local arm_seq = {{'move',nil, trRArmTarget1},{'wrist',nil, trRArmTarget2}}         
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "hookside" end
      end
    end
  elseif stage=="hookside" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        print("Current trRArm:",arm_planner.print_transform(trRArm))
        arm_planner:set_shoulder_yaw_target(-5*Body.DEG_TO_RAD, nil)          
        local arm_seq = {{'move',nil,{0.31,-0.60,-0.09, trRArmSidePush[4],trRArmSidePush[5],trRArmSidePush[6]}}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armsidepush" end
      end    
    end
  elseif stage=="armsidepush" then
    if arm_planner:play_arm_sequence(t) then 
      --arm_planner:set_shoulder_yaw_target(-5*Body.DEG_TO_RAD, 5*Body.DEG_TO_RAD)          
      local arm_seq = {
        {'move',nil,{0.30,-0.35,-0.09, trRArmSidePush[4],trRArmSidePush[5],trRArmSidePush[6]}},
        {'wrist',nil,trRArm0},
      }
      if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armbacktoinitpos1" end
    end










    
  elseif stage=="armbacktoinitpos1" then
    if arm_planner:play_arm_sequence(t) then 
      arm_planner:set_shoulder_yaw_target(-5*Body.DEG_TO_RAD, 5*Body.DEG_TO_RAD)          
      local arm_seq = {{'move',nil,trRArm0}}
      if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armbacktoinitpos" end
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
  hcm.set_state_success(1) --Report success
--  flush_debugdata()
  print(state._NAME..' Exit' )
end



return state
