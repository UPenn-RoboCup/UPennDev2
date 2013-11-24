local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()

--Door opening state using HOOK

local lhand_rpy0 = Config.armfsm.doorsupport.lhand_rpy
local lhand_rpy2 = Config.armfsm.doorsupport.lhand_rpy2
local rhand_rpy0 = Config.armfsm.dooredge.rhand_rpy



local edge_clearance = Config.armfsm.dooredge.edge_clearance
local door_yaw1 = Config.armfsm.dooredge.door_yaw1
local door_yaw2 = Config.armfsm.dooredge.door_yaw2


local trRArmRelease = {0,0,0,unpack(Config.armfsm.dooropen.rhand_rpy_release)}
local trRArmForward = {0,0,0,unpack(Config.armfsm.dooropen.rhand_rpy_forward)}
local trRArmSidePush = {0,0,0,unpack(Config.armfsm.dooropen.rhand_rpy_sidepush)}




local trLArm0, trRArm0, trLArm1, trRArm1
local stage
local debugdata

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_lhandoffset(Config.arm.handoffset.chopstick)
  mcm.set_arm_rhandoffset(Config.arm.handoffset.outerhook)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  qLArm0 = qLArm
  qRArm0 = qRArm
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  
  
  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

  hcm.set_door_model(Config.armfsm.dooropen.default_model)

  -- Default shoulder yaw angle: (-5,5)

--[[  
  arm_planner:set_shoulder_yaw_target(qLArm0[3],nil) --Lock left shoulder yaw
  local wrist_seq = {{'wrist',nil,trRArm1}}
  if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "wristyawturn"end
--]]

  arm_planner:set_shoulder_yaw_target(nil,nil) --Lock left shoulder yaw
  local wrist_seq = {{'wrist',trLArm1,trRArm1}}
  if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "wristyawturn"end



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
  
  local cur_cond = arm_planner:load_boundary_condition()
  local trLArm = Body.get_forward_larm(cur_cond[1])
  local trRArm = Body.get_forward_rarm(cur_cond[2])  

  if stage=="wristyawturn" then --Turn wrist angles without moving arms
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then        
--        arm_planner:set_shoulder_yaw_target(qLArm0[3],nil) --Lock left shoulder yaw

        arm_planner:set_shoulder_yaw_target(nil,nil)
        print("trLArm:",arm_planner.print_transform(trLArm))
        print("trRArm:",arm_planner.print_transform(trRArm))

        local trLArmTarget1 = Config.armfsm.doorsupport.arminit[1]        
        local trLArmTarget2 = {0,0,0,unpack(lhand_rpy2)}

        local trRArmTarget1 = movearm.getDoorEdgePosition(edge_clearance, door_yaw1, rhand_rpy0)        

        local arm_seq = {{'move',trLArmTarget1, trRArmTarget1},
                          {'wrist',trLArmTarget2, nil}}
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


        print("trLArm:",arm_planner.print_transform(trLArm))
        local trLArmTarget1 = Config.armfsm.doorsupport.arminit[2]        
        local trLArmTarget2 = Config.armfsm.doorsupport.arminit[3]
        local trRArmTarget1 = movearm.getDoorEdgePosition({0,0,0}, door_yaw1, rhand_rpy0)


        --local arm_seq = {{'move',nil, trRArmTarget1}}
        local arm_seq = {{'move',trLArmTarget1, trRArmTarget1},
            {'move',trLArmTarget2, nil},
            }
        if arm_planner:plan_arm_sequence2(arm_seq) then 
          stage = "hookknob"  
          else print("Xxx")
        end


      elseif hcm.get_state_proceed()==-1 then        
        local arm_seq = {{'move',nil,trRArm1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "wristyawturn" end        

      elseif hcm.get_state_proceed()==2 then --adjust hook position
        update_model()
        local trRArmTarget1 = movearm.getDoorEdgePosition(edge_clearance, door_yaw1, rhand_rpy0)
        local arm_seq = {{'move',nil,trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "placehook"  end

      end
    end
  elseif stage=="hookknob" then --Move up the hook to make it touch the knob
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then --Open the door
        arm_planner:save_doorparam({{0,0,0},door_yaw1,0,0})
        local dooropen_seq =         {
            {'dooredge',{0,0,0}, door_yaw1,0,0},
            {'dooredge',{0,0,0}, door_yaw2,0,0}            
          }
        if arm_planner:plan_arm_sequence2(dooropen_seq) then stage = "opendoor"  end

      elseif hcm.get_state_proceed()==-1 then --Lower the hook
        local trRArmTarget1 = movearm.getDoorEdgePosition(edge_clearance, door_yaw1, rhand_rpy0)
        local arm_seq = {{'move',nil,trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "placehook"  end

      elseif hcm.get_state_proceed()==2 then --adjust hook position
        update_model()
        local trRArmTarget1 = movearm.getDoorEdgePosition({0,0,0}, door_yaw1, rhand_rpy0)
        local arm_seq = {{'move',nil,trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "hookknob"  end

      end
    end
  elseif stage=="opendoor" then --Move the arm forward using IK now     
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==-1 then --Lower the hook
        local dooropen_seq = {{'dooredge',{0,0,0}, door_yaw1,0,0}}
        if arm_planner:plan_arm_sequence2(dooropen_seq) then stage = "hookknob"  end
      end
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

function state.exit()  
  hcm.set_state_success(1) --Report success
--  flush_debugdata()
  print(state._NAME..' Exit' )
end



return state
