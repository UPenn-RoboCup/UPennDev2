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
local lhand_rpy = Config.armfsm.toolgrip.lhand_rpy
local rhand_rpy = Config.armfsm.toolgrip.rhand_rpy


local gripL, gripR = 1,1
local stage
local debugdata

local function get_tool_tr(tooloffset)
  local handrpy = rhand_rpy
  local tool_model = hcm.get_tool_model()
  local hand_pos = vector.slice(tool_model,1,3) + 
    vector.new({tooloffset[1],tooloffset[2],tooloffset[3]})  
  local tool_tr = {hand_pos[1],hand_pos[2],hand_pos[3],
                    handrpy[1],handrpy[2],handrpy[3] + tool_model[4]}
  return tool_tr
end

local function get_hand_tr(pos)
  return {pos[1],pos[2],pos[3], unpack(rhand_rpy)}
end

local function update_model()
  local trRArmTarget = hcm.get_hands_right_tr_target()
  local trRArm = hcm.get_hands_right_tr()
  local tool_model = hcm.get_tool_model()
  tool_model[1],tool_model[2],tool_model[3], tool_model[4] = 
  tool_model[1] + trRArmTarget[1] - trRArm[1],
  tool_model[2] + trRArmTarget[2] - trRArm[2],
  tool_model[3] + trRArmTarget[3] - trRArm[3],
  tool_model[4] + util.mod_angle(trRArmTarget[6] - trRArm[6])

  hcm.set_tool_model(tool_model)
  print("Tool model:",tool_model[1],tool_model[2],tool_model[3])
  hcm.set_state_proceed(0)
end

local function update_override()
  local overrideTarget = hcm.get_state_override_target()
  local override = hcm.get_state_override()
  local tool_model = hcm.get_tool_model()

  tool_model[1],tool_model[2],tool_model[3], tool_model[4] = 
  tool_model[1] + (overrideTarget[1]-override[1]),
  tool_model[2] + (overrideTarget[2]-override[2]),
  tool_model[3] + (overrideTarget[3]-override[3]),
  tool_model[4] + (util.mod_angle(overrideTarget[4]-override[4]))

  hcm.set_tool_model(tool_model)
  print( util.color('Tool model:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f",
        tool_model[1],tool_model[2],tool_model[3],
        tool_model[4]*180/math.pi ))
  hcm.set_state_proceed(0)
end

local function revert_override()
  local overrideTarget = hcm.get_state_override_target()
  local override = hcm.get_state_override()
  local tool_model = hcm.get_tool_model()

  tool_model[1],tool_model[2],tool_model[3], tool_model[4] = 
  tool_model[1] - (overrideTarget[1]-override[1]),
  tool_model[2] - (overrideTarget[2]-override[2]),
  tool_model[3] - (overrideTarget[3]-override[3]),
  tool_model[4] - (util.mod_angle(overrideTarget[4]-override[4]))

  hcm.set_tool_model(tool_model)
  print( util.color('Tool model:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f",
        tool_model[1],tool_model[2],tool_model[3],
        tool_model[4]*180/math.pi ))
  hcm.set_state_proceed(0)
end

local function confirm_override()
  local override = hcm.get_state_override()
  hcm.set_state_override_target(override)
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_rhandoffset(Config.arm.handoffset.gripper)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  qLArm0 = qLArm
  qRArm0 = qRArm
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

  --Initial arm joint angles after rotating wrist
  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy)})
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy)})
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

  local trLArm05 = {unpack(trLArm0)}
  local trRArm05 = {unpack(trRArm0)}
  trLArm05[5] = trLArm1[5]
  trRArm05[5] = trRArm1[5]

  arm_planner:set_hand_mass(0,0)
  arm_planner:set_shoulder_yaw_target(qLArm0[3], nil) 

  local wrist_seq = {
    {'wrist',trLArm05, trRArm05},
    {'wrist',trLArm1, trRArm1},
  }

  --local wrist_seq = {{'wrist',trLArm1,trRArm1}}
  if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "wristyawturn" end  

    hcm.set_tool_model(Config.armfsm.toolgrip.default_model)
  hcm.set_state_proceed(1)

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
  
----------------------------------------------------------
--Forward motions
----------------------------------------------------------

  if stage=="wristyawturn" then --Turn yaw angles first    
    if arm_planner:play_arm_sequence(t) then       
      if hcm.get_state_proceed()==1 then 
--        print("trLArm:",arm_planner.print_transform(trLArm))
        print("trRArm:",arm_planner.print_transform(trRArm))
        arm_planner:set_shoulder_yaw_target(qLArm0[3],nil)        
        local arm_seq = {
          {'move',nil,Config.armfsm.toolgrip.arminit[1]},
          {'move',nil,Config.armfsm.toolgrip.arminit[2]},
          {'move',nil,Config.armfsm.toolgrip.arminit[3]},
          {'wrist',nil,Config.armfsm.toolgrip.arminit[4]},
          {'move',nil,Config.armfsm.toolgrip.arminit[4]}
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armup" end
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3]) 
        --local wrist_seq = {{"wrist",nil,trRArm0}}
        local wrist_seq = {{"wrist",trLArm0,trRArm0}}
        if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "armbacktoinitpos" end  
      end
    end
  elseif stage=="armup" then
    if arm_planner:play_arm_sequence(t) then 
          --Open gripper
      Body.move_rgrip1(Config.arm.torque.open)
      Body.move_rgrip2(Config.arm.torque.open)    
      
      if hcm.get_state_proceed()==1 then 
        print("trRArm:",arm_planner.print_transform(trRArm))
        arm_planner:set_shoulder_yaw_target(qLArm0[3],nil)
      

        local trRArmTarget1 = get_tool_tr(Config.armfsm.toolgrip.tool_clearance)        
        trRArmTarget1[1]=Config.armfsm.toolgrip.tool_clearance_x
        local trRArmTarget2 = get_tool_tr(Config.armfsm.toolgrip.tool_clearance)


        local arm_seq = {
          {'move',nil, trRArmTarget1},
          {'move',nil, trRArmTarget2}
        }     
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "reachout" end          
      elseif hcm.get_state_proceed()==-1 then 
        trRArmTarget1 = get_hand_tr(Config.armfsm.toolgrip.arminit[1])
        local arm_seq = {{'move',nil,trRArmTarget1},{'move',nil,trRArm1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "wristyawturn" end  
      end
    end        
  elseif stage=="reachout" then --Move arm to the gripping position (with clearance)
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        local trRArmTarget1 = get_tool_tr({0,0,0})        
        local arm_seq = {{'move',nil, trRArmTarget1}}     
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "touchtool" end        
      elseif hcm.get_state_proceed() == -1 then        
        local trRArmTarget0 = trRArm
        trRArmTarget0[1]=Config.armfsm.toolgrip.tool_clearance_x
        local trRArmTarget1 = get_hand_tr(Config.armfsm.toolgrip.arminit[2])
        local arm_seq={{'move',nil,trRArmTarget0},{'move',nil,trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armup" end
--[[  
      elseif hcm.get_state_proceed() == 2 then --Model modification
        update_model()        
        arm_planner:set_hand_mass(0,0)
        local trRArmTarget1 = get_tool_tr(Config.armfsm.toolgrip.tool_clearance)
        local arm_seq = {{'move',nil,trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "reachout" end
--]]        
      elseif hcm.get_state_proceed() == 3 then 
        update_override()        
        arm_planner:set_hand_mass(0,0)
        local trRArmTarget1 = get_tool_tr(Config.armfsm.toolgrip.tool_clearance)
        local arm_seq = {{'move',nil,trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then 
          stage = "reachout" 
          confirm_override()
        else revert_override() end
      end
    end    
  elseif stage=="touchtool" then --Move arm to the gripping position
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        local trRArmTarget2 = get_tool_tr({0,0,0})
        local arm_seq = {{'move',nil,trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "grab" end
      elseif hcm.get_state_proceed() == -1 then 
        arm_planner:set_hand_mass(0,0)
        local trRArmTarget1 = get_tool_tr(Config.armfsm.toolgrip.tool_clearance)
        local arm_seq = {{'move',nil, trRArmTarget1}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "reachout" end
      elseif hcm.get_state_proceed() == 2 then 
        arm_planner:set_hand_mass(0,0)
        update_model()        
        local trRArmTarget2 = get_tool_tr({0,0,0})        
        local arm_seq = {{'move',nil,trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "touchtool" end

      elseif hcm.get_state_proceed() == 3 then 
        arm_planner:set_hand_mass(0,0)
        update_override()        
        local trRArmTarget2 = get_tool_tr({0,0,0})        
        local arm_seq = {{'move',nil,trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then 
          stage = "touchtool" 
          confirm_override()
        else revert_override() end


      end
    end
    hcm.set_state_proceed(0)--stop at next step
  elseif stage=="grab" then --Grip the object   
    Body.move_rgrip1(Config.arm.torque.grip_drill)    
    Body.move_rgrip2(Config.arm.torque.grip_drill)    
    stage = "torsobalance"

  elseif stage=="torsobalance" then
    if arm_planner:play_arm_sequence(t) then    
      if hcm.get_state_proceed()==1 then        
        arm_planner:set_hand_mass(0,1)
        local trRArmTarget3 = get_tool_tr(Config.armfsm.toolgrip.tool_liftup)
        local arm_seq = {{'move',nil, trRArmTarget3}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "lift" end
      elseif hcm.get_state_proceed()==-1 then stage="ungrab" 
      elseif hcm.get_state_proceed() == 2 then --Model modification
        update_model()        
        local trRArmTarget2 = get_tool_tr({0,0,0})
        local arm_seq = {{'move',nil,trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "torsobalance" end
      elseif hcm.get_state_proceed() == 3 then --Model modification
        update_override()        
        local trRArmTarget2 = get_tool_tr({0,0,0})
        local arm_seq = {{'move',nil,trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "torsobalance" 
        confirm_override()
        else revert_override() end


      end
    end
    hcm.set_state_proceed(0) --stop here
  elseif stage=="lift" then
    if arm_planner:play_arm_sequence(t) then    
      if hcm.get_state_proceed()==1 then        
        arm_planner:set_hand_mass(0,2)
        print("trRArm:",arm_planner.print_transform(trRArm))
        local trRArmTarget1 = Config.armfsm.toolgrip.armpull[1]
        trRArmTarget1[2]=trRArm[2]
        trRArmTarget1[3]=trRArm[3]
        local trRArmTarget2 = Config.armfsm.toolgrip.armpull[2]
        trRArmTarget2[3]=trRArm[3]
        local trRArmTarget3 = Config.armfsm.toolgrip.armpull[3]
        local trRArmTarget4 = get_hand_tr(Config.armfsm.toolgrip.armhold)

        arm_planner:set_hand_mass(0,2)   
        local arm_seq = {
          {'move',nil,trRArmTarget1},
          {'move',nil,trRArmTarget2},
          {'move',nil,trRArmTarget3},
          {'move',nil,trRArmTarget4}          
        }

        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "liftpull" end
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_hand_mass(0,1)   
        local trRArmTarget3 = get_tool_tr({0,0,0})
        local arm_seq = {{'move',nil,trRArmTarget3}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "torsobalance" end
      elseif hcm.get_state_proceed() == 2 then --Model modification
        update_model()        
        local trRArmTarget2 = get_tool_tr(Config.armfsm.toolgrip.tool_liftup)
        local arm_seq = {{'move',nil,trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "lift" end
      elseif hcm.get_state_proceed() == 3 then --Model modification
        update_override()        
        local trRArmTarget2 = get_tool_tr(Config.armfsm.toolgrip.tool_liftup)
        local arm_seq = {{'move',nil,trRArmTarget2}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "lift" 
        confirm_override()
        else revert_override() end
      end
    end
  elseif stage=="liftpull" then --Move arm back to holding position
    if arm_planner:play_arm_sequence(t) then    
      stage = "pulldone"
      print("SEQUENCE DONE")
      return"hold"      
    end      

----------------------------------------------------------
--Backward motions motions
----------------------------------------------------------

  elseif stage=="ungrab" then --Ungrip the object
           --Open gripper
    Body.move_rgrip1(Config.arm.torque.open)
    Body.move_rgrip2(Config.arm.torque.open)   
    
    arm_planner:set_hand_mass(0,0)   
    local trRArmTarget2 = get_tool_tr({0,0,0})
    local arm_seq = {{'move',nil, trRArmTarget2}}
    if arm_planner:plan_arm_sequence2(arm_seq) then stage = "touchtool" end
  
  elseif stage=="armbacktoinitpos" then 
    if arm_planner:play_arm_sequence(t) then return "done" end
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
