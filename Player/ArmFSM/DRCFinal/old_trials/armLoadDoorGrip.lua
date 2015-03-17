local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()

--Door opening state using HOOK

local lhand_rpy0 = Config.armfsm.dooropen.lhand_support[1]
local rhand_rpy0 = Config.armfsm.dooropen.rhand_rpy
local rollTarget,yawTarget = 0,0

local trLArm0, trRArm0, trLArm1, trRArm1, trRArm2
local stage
local debugdata

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
  door_model[7] = math.max(-90*DEG_TO_RAD,math.min(0,door_model[7]))

  --Door yaw : plus to pull, minus to push
  door_model[8] = math.max(-30*DEG_TO_RAD,math.min(30*DEG_TO_RAD,door_model[8]))  

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

  qLArm1 = qLArm
  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, Config.armfsm.dooropen.lhand_support[1])

  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)

  qRArm2 = Body.get_inverse_arm_given_wrist( qRArm, Config.armfsm.dooropen.rhand_release[4])  
  trRArm2 = Body.get_forward_rarm(qRArm2)

  -- Default shoulder yaw angle: (-5,5)
--  arm_planner:set_shoulder_yaw_target(qLArm0[3],nil) --Lock left shoulder yaw
  arm_planner:set_shoulder_yaw_target(nil,nil) --Lock left shoulder yaw
  local wrist_seq = {{'wrist',trLArm1,trRArm1}}
  if arm_planner:plan_arm_sequence2(wrist_seq) then 
    stage = "wristyawturn"
    hcm.set_state_proceed(1)
  end
 
  hcm.set_door_model(Config.armfsm.dooropen.default_model)
  hcm.set_door_yaw(0)
  debugdata=''

  hcm.set_state_tstartactual(unix.time()) 
  hcm.set_state_tstartrobot(Body.get_time())

end



function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  if plan_failed then return "planfail" end
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  local door_yaw = hcm.get_door_yaw()
  local cur_cond = arm_planner:load_boundary_condition()
  local qLArm = cur_cond[1]
  local qRArm = cur_cond[2]
  local trLArm = Body.get_forward_larm(cur_cond[1])
  local trRArm = Body.get_forward_rarm(cur_cond[2])  

  if stage=="wristyawturn" then --Turn wrist angles without moving arms
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then        
        print("trLArm:",util.print_transform(trLArm))        
        local trRArmTarget1 = movearm.getDoorHandlePosition({0,0,0}, 0, 0)
        local arm_seq = {
          {'move',Config.armfsm.dooropen.lhand_support[2],trRArmTarget1},
          {'wrist',Config.armfsm.dooropen.lhand_support[3],nil},
          {'move',Config.armfsm.dooropen.lhand_support[4],nil},
          {'move',Config.armfsm.dooropen.lhand_support[5],nil},
        } 
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "opendoor"  end
        hcm.set_state_proceed(0)
      elseif hcm.get_state_proceed()==-1 then
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3])
        local wrist_seq = {{'wrist',nil,trRArm0}}
        if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "armbacktoinitpos" end   
      end
    end  
  elseif stage=="opendoor" then --Move the hook below the door knob
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then
        print("trLArm:",util.print_transform(trLArm))        
--      arm_seq={{'move',Config.armfsm.dooropen.lhand_support[6],nil}}

        --Rotate waist and support with left hand
        arm_seq={
          {'move',nil,nil,Config.armfsm.dooropen.waistTarget},
--          {'move',Config.armfsm.dooropen.lhand_support[6],nil}
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "hooksupport" end
        hcm.set_state_proceed(0)
      elseif hcm.get_state_proceed()==-1 then --Re-hook

      elseif hcm.get_state_proceed()==3 then --move arms around
        update_override()
        local trRArmTarget1 = movearm.getDoorHandlePosition({0,0,0}, rollTarget, yawTarget)
        local arm_seq = {{'move',nil,trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "opendoor"  end
      end
    end
  elseif stage=="hooksupport" then 
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        local trRArmTarget1 = movearm.getDoorHandlePosition(Config.armfsm.dooropen.handle_clearance2, 0, yawTarget)
        local arm_seq = {{'move',nil,trRArmTarget1}}

local arm_seq = {{'move',Config.armfsm.dooropen.lhand_support[6],nil}}
hcm.set_state_proceed(0)
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "hookrelease" end
      end
    end
  elseif stage=="hookrelease" then     
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then        
        local trArmRelease0 = Config.armfsm.dooropen.rhand_release
        trArmRelease0[6] = trRArm[6]        
        local wrist_seq = {
          {'wrist',nil, trRArmRelease0},
          {'wrist',nil, Config.armfsm.dooropen.rhand_release[2]},
          {'wrist',nil, Config.armfsm.dooropen.rhand_release[3]},
        }        

        if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "hookrollback"  
        else hcm.set_state_proceed(0) end
      elseif hcm.get_state_proceed()==2 then
        update_model()
        local doorparam = arm_planner.init_doorparam
        local dooropen_seq ={{'door',Config.armfsm.dooropen.handle_clearance2,0*DEG_TO_RAD,doorparam[3]},}
        if arm_planner:plan_arm_sequence2(dooropen_seq) then stage = "hookrelease"  end
      end
    end    
  elseif stage=="hookrollback" then
    if arm_planner:play_arm_sequence(t) then       
      if hcm.get_state_proceed()==1 then
--print("trRArm:",util.print_transform(trRArm))
--        arm_planner:set_shoulder_yaw_target(qLArm0[3], qRArm0[3]) 
        local arm_seq={
          {'move',nil,{trRArm[1]-0.10,trRArm[2],trRArm[3],trRArm[4],trRArm[5],trRArm[6]}},
          {'wrist',nil, Config.armfsm.dooropen.rhand_release[4]},
          {'move',nil,trRArm2},          
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "hookforward" end      
        hcm.set_state_proceed(0)
      end
    end
  elseif stage=="hookforward" then    
    if arm_planner:play_arm_sequence(t) then      
      if hcm.get_state_proceed()==1 then
        local wrist_seq = {
          {'wrist',nil, Config.armfsm.dooropen.rhand_forward[1]},
          {'wrist',nil, Config.armfsm.dooropen.rhand_forward[2]},          
        }
        if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "sidepush" end
      end
    end    
  elseif stage=="sidepush" then        
    if arm_planner:play_arm_sequence(t) then      
      if hcm.get_state_proceed()==1 then
        print("trRArm:",util.print_transform(trRArm))        
        local wrist_seq = {         
          {'wrist',nil, Config.armfsm.dooropen.rhand_sidepush[1]},
        }
        if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "armreturn" end
      end
    end
  elseif stage=="armreturn" then
    if arm_planner:play_arm_sequence(t) then      
      local wrist_seq = {
        {'move',Config.armfsm.dooropen.lhand_support[5],nil},          
        {'move',Config.armfsm.dooropen.lhand_support[4],nil},                  
        {'move',nil,nil,0},--rotate waist
      }
      if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "armreturn2" end
    end




  elseif stage=="armreturn2" then
    if arm_planner:play_arm_sequence(t) then 
      print("trLArm:",util.print_transform(trLArm))              
      local wrist_seq = {
        {'move',Config.armfsm.dooropen.lhand_unsupport[1],nil},
        {'wrist',Config.armfsm.dooropen.lhand_unsupport[2],nil},
        {'move',Config.armfsm.dooropen.lhand_unsupport[2],nil},
      }
      if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "armbacktoinitpos" end
    end
  elseif stage=="armbacktoinitpos" then
    if arm_planner:play_arm_sequence(t) then 
--print("trLArm:",util.print_transform(trLArm))              
--          {'wrist',nil, Config.armfsm.dooropen.rhand_sidepush[3]},
--          {'move',nil, trRArm0}
   
    end
  end
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
