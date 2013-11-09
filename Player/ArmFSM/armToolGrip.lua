local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()

local gripL, gripR = 0,0
local stage = 1;
local debugdata
local plan_failed = false

local qLArm0,qRArm0, trLArm0, trRArm0

--Initial hand angle
local lhand_rpy0 = {0,0*Body.DEG_TO_RAD, -45*Body.DEG_TO_RAD}
local rhand_rpy0 = {0,0*Body.DEG_TO_RAD, 45*Body.DEG_TO_RAD}

local current_arm_plan, current_arm_endcond


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

  Body.set_lgrip_percent(0)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  --Initial arm joint angles after rotating wrist
  qLArm0 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})
  qRArm0 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})

  --Target hand position (for idle hand)
  trLArm0 = {0.10,0.24,-0.10, unpack(lhand_rpy0)}
  trRArm0 = {0.10,-0.24,-0.10, unpack(rhand_rpy0)}
 
  hcm.set_tool_model({0.55,0.02,0.05,  0*Body.DEG_TO_RAD}) --for webots with bodyTilt
  --hcm.set_tool_model({0.51,0.02,0.05,  0*Body.DEG_TO_RAD}) 
 
  local trLArmTarget1 = get_tool_tr({0,0.08,0}, lhand_rpy0)
  local trLArmTarget2 = get_tool_tr({0,0,0}, lhand_rpy0)

  --This sets torso compensation bias
  --So that it becomes zero with initial arm configuration
  arm_planner:reset_torso_comp(qLArm0, qRArm0)

  local arm_sequence1 = {
    init={qLArm0,qRArm0,qLArm0, qRArm0, {0,0}},
    mass={0,0},
    armseq={
      {trLArmTarget1, trRArm0},
      {trLArmTarget2, trRArm0},
    }
  }
  current_arm_plan, current_arm_endcond = arm_planner:plan_arm_sequence(arm_sequence1)

  stage = 1;  
  debugdata=''   
end


function state.update()
  --  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  if plan_failed then return "planfail" end

  if stage==1 then --Rotate wrist angles
    ret = movearm.setArmJoints(qLArm0, qRArm0 ,dt, Config.arm.joint_init_limit)
    if ret==1 then     
      arm_planner:init_arm_sequence(current_arm_plan,t)
      stage=stage+1 
    end

  elseif stage==2 then --Move arm to the gripping position
    if arm_planner:play_arm_sequence(t) then    
      stage = stage+1             
    end

  elseif stage==3 then --Wait for proceed confirmation
    if hcm.get_state_proceed()==1 then
      hcm.set_state_proceed(0)
      stage = stage+1

    elseif hcm.get_state_proceed() == -1 then
      hcm.set_state_proceed(0)
      stage = -2 --negative stage number means we are going backward
    end

  elseif stage==4 then --Grip the object   
    gripL,doneL = util.approachTol(gripL,1,2,dt)
    Body.set_lgrip_percent(gripL*0.8)
    Body.set_rgrip_percent(gripR*0.8)    
    if doneL then
      if hcm.get_state_proceed()==1 then
        hcm.set_state_proceed(0)
        stage=stage+1


        local trLArmTarget3 = get_tool_tr({0,0,0.05}, lhand_rpy0)
        local trLArmTarget4 = get_tool_tr({-0.20,0,0.05}, lhand_rpy0)
        local trLArmTarget5 = {0.20,0.0,-0.10, unpack(lhand_rpy0)}
        local arm_sequence2 = {
          init=current_arm_endcond,
          mass={3,0},
          armseq={
            {trLArmTarget3, trRArm0},
            {trLArmTarget4, trRArm0},
            {trLArmTarget5, trRArm0},
          }
        }
        current_arm_plan, current_arm_endcond = arm_planner:plan_arm_sequence(arm_sequence2)

        arm_planner:init_arm_sequence(current_arm_plan,t)
      end
    end
  elseif stage==5 then --Move arm back to holding position
    if arm_planner:play_arm_sequence(t) then    
      stage = stage+1
      print("SEQUENCE DONE")
      return"done"      
    end      
  end
end

function state.exit()  
  --Store boundary conditions for future state
  arm_planner:save_boundary_condition(current_arm_endcond)
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