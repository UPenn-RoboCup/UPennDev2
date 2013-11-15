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
local lhand_rpy0 = {0,0*Body.DEG_TO_RAD, -45*Body.DEG_TO_RAD}
local rhand_rpy0 = {0,0*Body.DEG_TO_RAD, 45*Body.DEG_TO_RAD}

local gripL, gripR = 1,1
local stage
local debugdata




local function get_tool_tr(tooloffset, handrpy)
  local tool_model = hcm.get_tool_model()
  local hand_pos = vector.slice(tool_model,1,3) + vector.new(tooloffset)  
  local tool_tr = {hand_pos[1],hand_pos[2],hand_pos[3],
                    handrpy[1],handrpy[2],handrpy[3] + tool_model[4]}
  return tool_tr
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_handoffset(Config.arm.handoffset.gripper)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  qLArm0 = qLArm
  qRArm0 = qRArm
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

--This sets torso compensation bias so that it becomes zero with initial arm configuration
  arm_planner:reset_torso_comp(qLArm, qRArm)
  arm_planner:save_boundary_condition({qLArm, qRArm, qLArm, qRArm, {0,0}})  

  
  --Initial arm joint angles after rotating wrist
  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

hcm.set_tool_model({0.52,0.02,0.00,  0*Body.DEG_TO_RAD}) --for webots with bodyTilt


print("XXX")
print(unpack(vector.new(qLArm0)*180/math.pi))
print(unpack(vector.new(qRArm0)*180/math.pi))


  arm_planner:set_shoulder_yaw_target(nil,qRArm0[3]) --Lock right shoulder yaw
  local wrist_seq = { armseq={ {trLArm1,trRArm0}} }
  if arm_planner:plan_wrist_sequence(wrist_seq) then stage = "wristyawturn" end  
  hcm.set_state_proceed(1)
  

  debugdata=''   
end

local function update_model()
  local trLArmTarget = hcm.get_hands_left_tr_target()
  local tool_model = hcm.get_tool_model()
  tool_model[1],tool_model[2],tool_model[3] = trLArmTarget[1],trLArmTarget[2],trLArmTarget[3]
  hcm.set_tool_model(tool_model)
end


function state.update()
  --  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t   -- Save this at the last update time
  
----------------------------------------------------------
--Forward motions
----------------------------------------------------------

  if stage=="wristyawturn" then --Turn yaw angles first    
    gripL,doneL = util.approachTol(gripL,1,2,dt)  --Close gripper
    Body.set_lgrip_percent(gripL*0.8)
    if arm_planner:play_arm_sequence(t) then       
      if hcm.get_state_proceed()==1 then 
        arm_planner:set_shoulder_yaw_target(nil,qRArm0[3])
        trLArmTarget1 = {0.25,0.20,-0.05, unpack(lhand_rpy0)}
        trLArmTarget2 = {0.25,0.20, -0.03, unpack(lhand_rpy0)}
        local arm_seq = {
          mass={0,0},
          armseq={{trLArmTarget1, trRArm0},{trLArmTarget2, trRArm0}}
        }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "armup" end
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3]) 
        local wrist_seq = { armseq={    {trLArm0,trRArm0}      }}
        if arm_planner:plan_wrist_sequence(wrist_seq) then stage = "armbacktoinitpos" end  
      end
    end
  elseif stage=="armup" then
    if arm_planner:play_arm_sequence(t) then stage = "initialwait" end


  elseif stage=="initialwait" then
    gripL,doneL = util.approachTol(gripL,0,2,dt)  --Open gripper
    Body.set_lgrip_percent(gripL*0.8)
    if doneL then
      if hcm.get_state_proceed()==1 then 
        arm_planner:set_shoulder_yaw_target(nil,qRArm0[3])
        local trLArmTarget1 = get_tool_tr({0,0.08,0}, lhand_rpy0)
        local trLArmTarget2 = get_tool_tr({0,0,0}, lhand_rpy0)
        local arm_seq = {armseq={{trLArmTarget1, trRArm0}, {trLArmTarget2, trRArm0} } }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "reachout" end
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3]) --Lock right shoulder yaw
        trLArmTarget1 = {0.25,0.20,-0.05, unpack(lhand_rpy0)}
        local arm_seq = { armseq={    {trLArmTarget1,trRArm0},{trLArm1,trRArm0}      }}
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "wristyawturn" end  
      end
    end 
  elseif stage=="reachout" then --Move arm to the gripping position
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then stage = "grab"
      elseif hcm.get_state_proceed() == -1 then 
        local trLArmTarget1 = get_tool_tr({0,0.08,0}, lhand_rpy0)
        local trLArmTarget2 = {0.35,0.20, trLArmTarget1[3], unpack(lhand_rpy0)}
        local trLArmTarget3 = {0.25,0.20,-0.05, unpack(lhand_rpy0)}
        local arm_seq = {      
          armseq={        
            {trLArmTarget1, trRArm0},        
            {trLArmTarget2, trRArm0},
            {trLArmTarget3, trRArm0},
          }
        }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "armup" end
      elseif hcm.get_state_proceed() == 2 then --Model modification
        update_model()        
        local trLArmTarget2 = get_tool_tr({0,0,0}, lhand_rpy0)
        local arm_seq = {armseq={ {trLArmTarget2, trRArm0 }}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "reachout" end
      end
    end
--[[    
    local qLArm = Body.get_larm_command_position()
    print(string.format("Wrist: %.3f %.3f %.3f",
      qLArm[5]*Body.RAD_TO_DEG,
      qLArm[6]*Body.RAD_TO_DEG,
      qLArm[7]*Body.RAD_TO_DEG))
--]]      
  elseif stage=="grab" then --Grip the object   
    gripL,doneL = util.approachTol(gripL,1,2,dt)
    Body.set_lgrip_percent(gripL*0.8)
    if doneL then
      local trLArmTarget2 = get_tool_tr({0,0,0}, lhand_rpy0)
      local arm_seq = {          
        mass={1,0}, --TODO: this is not working right now          
        armseq={ {trLArmTarget2, trRArm0} }
        }      
      if arm_planner:plan_arm_sequence(arm_seq) then stage = "torsobalance" end
    end
  elseif stage=="torsobalance" then
    if arm_planner:play_arm_sequence(t) then    
      if hcm.get_state_proceed()==1 then        
        local trLArmTarget3 = get_tool_tr({0,0,0.05}, lhand_rpy0)
        local arm_seq = {          
          mass={2,0}, --TODO: this is not working right now          
          armseq={ {trLArmTarget3, trRArm0} }
        }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "lift" end
      elseif hcm.get_state_proceed()==-1 then stage="ungrab" end
    end
  elseif stage=="lift" then
    if arm_planner:play_arm_sequence(t) then    
      if hcm.get_state_proceed()==1 then        
        local trLArmTarget4 = get_tool_tr({-0.20,0,0.05}, lhand_rpy0)
        local trLArmTarget5 = {0.20,0.0,-0.10, unpack(lhand_rpy0)}
        local arm_seq = {          
          mass={2,0}, --TODO: this is not working right now          
          armseq={
            {trLArmTarget4, trRArm0},
            {trLArmTarget5, trRArm0},
          }
        }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "liftpull" end
      elseif hcm.get_state_proceed()==-1 then 
        local trLArmTarget3 = get_tool_tr({0,0,0}, lhand_rpy0)
        local arm_seq = {          
          mass={1,0}, --TODO: this is not working right now          
          armseq={ {trLArmTarget3, trRArm0} }
        }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "torsobalance" end
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
    gripL,doneL = util.approachTol(gripL,0,2,dt)
    Body.set_lgrip_percent(gripL*0.8)
    Body.set_rgrip_percent(gripR*0.8)    
    if doneL then
      local trLArmTarget2 = get_tool_tr({0,0,0}, lhand_rpy0)
      local arm_seq = {          
        mass={0,0}, --TODO: this is not working right now          
        armseq={ {trLArmTarget2, trRArm0} }
        }      
      if arm_planner:plan_arm_sequence(arm_seq) then stage = "reachout" end
    end
  elseif stage=="unreachout" then
    
  elseif stage=="unreachoutmove" then 
    if arm_planner:play_arm_sequence(t) then stage = "initialwait" end
  elseif stage=="armbacktoinitpos" then 
    if arm_planner:play_arm_sequence(t) then return "done" end
  end

  hcm.set_state_proceed(0)
end

function state.exit()  
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