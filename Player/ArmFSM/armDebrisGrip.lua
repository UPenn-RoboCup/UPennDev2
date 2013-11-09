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

local arm_plan1, arm_end1
local arm_plan2, arm_end2
local qLArm0,qRArm0

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  trLArm = Body.get_forward_larm(qLArm)
  trRArm = Body.get_forward_larm(qRArm)

  --Initial hand angle
  local lhand_rpy0 = {0,0*Body.DEG_TO_RAD, -45*Body.DEG_TO_RAD}
  local rhand_rpy0 = {0,0*Body.DEG_TO_RAD, 45*Body.DEG_TO_RAD}

  --Initial arm joint angles
  qLArm0 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})
  qRArm0 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})

 
  tool_pos_left=vector.new({0.51,0.02,0.05})  --For webots with zero bodyTilt
  Body.set_lgrip_percent(0)
  
  local trLArmTarget1 = vector.new({0.51,0.02,0.05, unpack(lhand_rpy0)})

  --This sets torso compensation bias
  --So that it becomes zero with initial arm configuration
  arm_planner:reset_torso_comp(qLArm0, qRArm0)

  local arm_sequence1 = {
    init={qLArm0,qRArm,qLArm0, qRArm, {0,0}},
    mass={0,0},
    armseq={
      {trLArmTarget1, trRArm},      
    }
  }
  arm_plan1, arm_end1 = arm_planner:plan_arm_sequence(arm_sequence1)
 
  stage = 1;  
  debugdata='' 
  if not arm_plan2 then plan_failed = true end
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
      arm_planner:init_arm_sequence(arm_plan1,t)
      stage=stage+1 
    end
  elseif stage==2 then --Move arm to the gripping position
    if arm_planner:play_arm_sequence(t) then    
      stage = stage+1             
    end
  end


--[[    
  elseif stage==3 then --Grip the object   
    gripL,doneL = util.approachTol(gripL,1,2,dt)
    Body.set_lgrip_percent(gripL*0.8)
    Body.set_rgrip_percent(gripR*0.8)    
    if doneL then
      stage=stage+1
      arm_planner:init_arm_sequence(arm_plan2,t)
    end
  elseif stage==4 then --Move arm back to holding position
    if arm_planner:play_arm_sequence(t) then    
      print("SEQUENCE DONE")
      stage = stage+1      
    end    
  elseif stage==5 then


  end
  --]]
end

function state.exit()  
  --Store boundary conditions for future state
  arm_planner:save_boundary_condition(arm_end1)
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