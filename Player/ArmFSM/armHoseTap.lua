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

local stage
local debugdata

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_lhandoffset(Config.arm.handoffset.gripper)
  mcm.set_arm_rhandoffset(Config.arm.handoffset.chopstick)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  qLArm0 = qLArm
  qRArm0 = qRArm
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

  --Initial arm joint angles after rotating wrist
  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, Config.armfsm.hosetap.larminit[1])
  qRArm1 = qRArm
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

  arm_planner:set_hand_mass(0,0)
  arm_planner:set_shoulder_yaw_target(nil,qRArm0[3]) --Lock left hand
  --[[
  local wrist_seq = {
    {'wrist',trLArm1,nil},
  }
--]]

  local wrist_seq = {
    {'move',Config.armfsm.hosegrip.armhosepull[1],nil},
    {'move',Config.armfsm.hosegrip.armhosepull[2],nil},
    {'move',Config.armfsm.hosegrip.armhosepull[3],nil},
    {'wrist',Config.armfsm.hosegrip.armhosepull[4],nil},
    {'move',Config.armfsm.hosegrip.armhosepull[4],nil},
  }

  print("trLArm0:",arm_planner.print_transform(trLArm0))
  print("trLArm1:",arm_planner.print_transform(trLArm1))

  if arm_planner:plan_arm_sequence(wrist_seq) then stage = "wristyawturn" end
  hcm.set_state_proceed(1)
  hcm.set_state_proceed(0)

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
    if arm_planner:play_arm_sequence(t) then       
      if hcm.get_state_proceed()==1 then 
        print("trLArm:",arm_planner.print_transform(trLArm))
        print("trRArm:",arm_planner.print_transform(trRArm))
        --[[
        local arm_seq = {
          {'move',Config.armfsm.hosetap.larminit[2],nil},
          {'move',Config.armfsm.hosetap.larminit[3],nil},
          {'move',Config.armfsm.hosetap.larminit[4],nil},          
          {'move',Config.armfsm.hosetap.larminit[5],nil},          
        }                          
        --]]

        local arm_seq = {
          {'wrist',Config.armfsm.hosegrip.armhoseattachinit[1],nil},
          {'wrist',Config.armfsm.hosegrip.armhoseattachinit[2],nil},
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armup" end

      elseif hcm.get_state_proceed()==-1 then 
        local wrist_seq = {
          {'wrist',Config.armfsm.hosegrip.armhosepull[4],nil},
          {'move',Config.armfsm.hosegrip.armhosepull[4],nil},
          {'wrist',Config.armfsm.hosegrip.armhosepull[3],nil},
          {'move',Config.armfsm.hosegrip.armhosepull[3],nil},
          {'move',Config.armfsm.hosegrip.armhosepull[1],nil},
        }
--        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3]) 
--        local wrist_seq = {{"wrist",trLArm0,nil}}
        if arm_planner:plan_arm_sequence2(wrist_seq) then 
          stage = "armbacktoinitpos" 
          else print ("ERROR?") end  
      end
    end
  elseif stage=="armup" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then         
        hcm.set_state_proceed(0)
      elseif hcm.get_state_proceed()==-1 then 
        local arm_seq = {
          {'move',Config.armfsm.hosetap.larminit[4],nil},
          {'move',Config.armfsm.hosetap.larminit[3],nil},
          {'move',Config.armfsm.hosetap.larminit[2],nil},          
          {'move',Config.armfsm.hosetap.larminit[1],nil},          
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "wristyawturn" end  
      elseif hcm.get_state_proceed()==3 then 

        local trLArmCurrent = hcm.get_hands_left_tr()
        local overrideTarget = hcm.get_state_override_target()
        local override = hcm.get_state_override()
        local trLArmTarget = {
          trLArmCurrent[1]+overrideTarget[1]-override[1],
          trLArmCurrent[2]+overrideTarget[2]-override[2],
          trLArmCurrent[3]+overrideTarget[3]-override[3],
          trLArmCurrent[4],
          trLArmCurrent[5],
          trLArmCurrent[6]
        }
        local arm_seq = {{'move',trLArmTarget,nil}}
        if arm_planner:plan_arm_sequence2(arm_seq) then
          stage = "armup"
        end
        hcm.set_state_proceed(0)
      end
    end 
   elseif stage=="reachout" then
    if arm_planner:play_arm_sequence(t) then 

    end
   elseif stage=="armbacktoinitpos" then
    if arm_planner:play_arm_sequence(t) then 
      return "hold"   
    end
   end

  
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