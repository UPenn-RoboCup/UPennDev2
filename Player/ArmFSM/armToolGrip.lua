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

  --Initial hand angle
  local lhand_rpy0 = {0,0*Body.DEG_TO_RAD, -45*Body.DEG_TO_RAD}
  local rhand_rpy0 = {0,0*Body.DEG_TO_RAD, 45*Body.DEG_TO_RAD}

  --Initial arm joint angles
  qLArm0 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})
  qRArm0 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})

  --Target hand position (for idle hand)
  local trLArm0 = {0.10,0.24,-0.10, unpack(lhand_rpy0)}
  local trRArm0 = {0.10,-0.24,-0.10, unpack(rhand_rpy0)}
 
  --tool_pos_left = hcm.get_tool_pos()
  tool_pos_left=vector.new({0.55,0.02,0.05})  
  tool_pos_left=vector.new({0.51,0.02,0.05})  --For webots with zero bodyTilt

  tool_yaw = 0 --TODO

  Body.set_lgrip_percent(0)
  


  local tool_pos_left1=tool_pos_left + vector.new({0,0,0.05})
  local tool_pos_left2=vector.new({0.35,0.05,tool_pos_left1[3]})  
  local tool_pos_left3=vector.new({0.20,0.0,-0.10})  

  local trLArmTarget1 = movearm.getToolPosition(tool_pos_left,0.08,1)    
  local trLArmTarget2 = movearm.getToolPosition(tool_pos_left,0,1)    
  local trLArmTarget3 = movearm.getToolPosition(tool_pos_left1,0,1)   --lift up 
  local trLArmTarget4 = movearm.getToolPosition(tool_pos_left2,0,1)    
  local trLArmTarget5 = movearm.getToolPosition(tool_pos_left3,0,1)    


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
  arm_plan1, arm_end1 = arm_planner:plan_arm_sequence(arm_sequence1)

  local arm_sequence2 = {
    init=arm_end1,
    mass={3,0},
    armseq={
      {trLArmTarget3, trRArm0},
      {trLArmTarget4, trRArm0},
      {trLArmTarget5, trRArm0},
    }
  }
  arm_plan2,arm_end2 = arm_planner:plan_arm_sequence(arm_sequence2)

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
end

function state.exit()  
  --Store boundary conditions for future state
  arm_planner:save_boundary_condition(arm_end2)
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