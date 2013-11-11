local state = {}
state._NAME = ...
require'hcm'
require'mcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()

local qLArm0,qRArm0, trLArm0, trRArm0

--Initial hand angle
local lhand_rpy0 = {0,0*Body.DEG_TO_RAD, -45*Body.DEG_TO_RAD}
local rhand_rpy0 = {0,0*Body.DEG_TO_RAD, 45*Body.DEG_TO_RAD}

local gripL, gripR = 0,0
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

  Body.set_lgrip_percent(0)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  --Initial arm joint angles after rotating wrist
  qLArm0 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})
  qRArm0 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
 
  hcm.set_tool_model({0.55,0.02,0.05,  0*Body.DEG_TO_RAD}) --for webots with bodyTilt
  --hcm.set_tool_model({0.51,0.02,0.05,  0*Body.DEG_TO_RAD}) 
   
  --This sets torso compensation bias so that it becomes zero with initial arm configuration
  arm_planner:reset_torso_comp(qLArm0, qRArm0)
  arm_planner:save_boundary_condition({qLArm0, qRArm0, qLArm0, qRArm0, {0,0}})

  stage = "wristrotate";  
  debugdata=''   
end

function state.update()
  --  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t   -- Save this at the last update time
  
----------------------------------------------------------
--Forward motions
----------------------------------------------------------

  if stage=="wristrotate" then --Rotate wrist angles
    ret = movearm.setArmJoints(qLArm0, qRArm0 ,dt, Config.arm.joint_init_limit)
    if ret==1 then 
      --Both hands around waist
      trLArm0 = {0.10,0.24,-0.10, unpack(lhand_rpy0)}
      trRArm0 = {0.10,-0.24,-0.10, unpack(rhand_rpy0)}
      local arm_seq = {
        mass={0,0},
        armseq={
          {trLArm0, trRArm0},          
        }
      }
      if arm_planner:plan_arm_sequence(arm_seq) then stage = "armup" end
    end
  elseif stage=="armup" then
    if arm_planner:play_arm_sequence(t) then stage = "initialwait" end
  elseif stage=="initialwait" then
    if hcm.get_state_proceed()==1 then 
      local trLArmTarget1 = get_tool_tr({0,0.08,0}, lhand_rpy0)
      local trLArmTarget2 = get_tool_tr({0,0,0}, lhand_rpy0)
      local arm_seq = {
        mass={0,0},
        armseq={
          {trLArmTarget1, trRArm0},
          {trLArmTarget2, trRArm0},
        }
      }
      if arm_planner:plan_arm_sequence(arm_seq) then stage = "reachout" end
    end
  elseif stage=="reachout" then --Move arm to the gripping position
    if arm_planner:play_arm_sequence(t) then stage = "reachwait" end
  elseif stage=="reachwait" then --Wait for proceed confirmation
    if hcm.get_state_proceed()==1 then stage = "grab"
    elseif hcm.get_state_proceed() == -1 then stage = "unreachout" 
    end
  elseif stage=="grab" then --Grip the object   
    gripL,doneL = util.approachTol(gripL,1,2,dt)
    Body.set_lgrip_percent(gripL*0.8)
    Body.set_rgrip_percent(gripR*0.8)    
    if doneL then
      if hcm.get_state_proceed()==1 then        
        local trLArmTarget3 = get_tool_tr({0,0,0.05}, lhand_rpy0)
        local trLArmTarget4 = get_tool_tr({-0.20,0,0.05}, lhand_rpy0)
        local trLArmTarget5 = {0.20,0.0,-0.10, unpack(lhand_rpy0)}
        local arm_seq = {          
          mass={3,0},
          armseq={
            {trLArmTarget3, trRArm0},
            {trLArmTarget4, trRArm0},
            {trLArmTarget5, trRArm0},
          }
        }
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "pull" end
      elseif hcm.get_state_proceed()==-1 then stage="ungrab" end
    end
  elseif stage=="pull" then --Move arm back to holding position
    if arm_planner:play_arm_sequence(t) then    
      stage = "pulldone"
      print("SEQUENCE DONE")
      return"done"      
    end      

----------------------------------------------------------
--Backward motions motions
----------------------------------------------------------

  elseif stage=="ungrab" then --Ungrip the object
    gripL,doneL = util.approachTol(gripL,0,2,dt)
    Body.set_lgrip_percent(gripL*0.8)
    Body.set_rgrip_percent(gripR*0.8)    
    if doneL then
      if hcm.get_state_proceed()==1 then stage= "grab"
      elseif hcm.get_state_proceed()==-1 then stage = "unreachout"
      end
    end
  elseif stage=="unreachout" then
    local trLArmTarget1 = get_tool_tr({0,0.08,0}, lhand_rpy0)
    local trLArmTarget2 = get_tool_tr({0,0,0}, lhand_rpy0)
    local arm_seq = {      
      mass={0,0},
      armseq={
        {trLArmTarget2, trRArm0},
        {trLArmTarget1, trRArm0},        
        {trLArm0, trRArm0},        
      }
    }
    if arm_planner:plan_arm_sequence(arm_seq) then stage = "unreachoutmove" end
  elseif stage=="unreachoutmove" then 
    if arm_planner:play_arm_sequence(t) then stage = "initialwait" end
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