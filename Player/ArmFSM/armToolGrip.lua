local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'

local dqArmMax = Config.arm.slow_limit
local dpArmMax = Config.arm.linear_slow_limit
local stage = 1;

local qLArm, qRArm, trLArm, trRArm, trTarget
local tool_pos,tool_pos_target

local qLArmTarget0, qRArmTarget0
local trLArmTarget,trRArmTarget

local libArmPlan = require 'libArmPlan'
arm_planner = libArmPlan.new_planner()

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  --open gripper
  Body.set_lgrip_percent(0)
  Body.set_rgrip_percent(0)

  qLArm = Body.get_larm_command_position()
  qRArm = Body.get_rarm_command_position()
  trLArm = Body.get_forward_larm(qLArm);
  trRArm = Body.get_forward_rarm(qRArm)  

   
  --tool_pos_left = hcm.get_tool_pos()
  tool_pos_left=vector.new({0.45,0.0,0.10})  
  tool_pos_right0=vector.new({0.45,-0.25,0.15})
  tool_pos_right=vector.new({0.45,-0.10,0.15})

  stage = 1;  

  trLArmTarget0 = trLArm
  trLArmTarget1 = movearm.getToolPosition(tool_pos_left,0.08,1)    
  trLArmTarget2 = movearm.getToolPosition(tool_pos_left,0,1)    
  print("Planning LArmPlan1")
  LArmPlan1,qLArm1 = arm_planner:plan_arm(qLArm, trLArmTarget1, 1)  
  print("Planning LArmPlan2")
  LArmPlan2,qLArm2 = arm_planner:plan_arm(qLArm1, trLArmTarget2, 1)

  arm_planner:init_trajectory(LArmPlan1,t_entry)
end

local gripL, gripR = 0,0

local qJointVelTool = 
  {30*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,
   30*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD}

function state.update()
  --  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  qLArm = Body.get_larm_command_position()
  qRArm = Body.get_rarm_command_position()
  trLArm = Body.get_forward_larm(qLArm)
  trRArm = Body.get_forward_rarm(qRArm)


  if stage==1 then
    local qLArmTarget = arm_planner:playback_trajectory(t)
    if qLArmTarget then movearm.setArmJoints(qLArmTarget,qRArm,dt)
    else 
      stage = stage+1 
      arm_planner:init_trajectory(LArmPlan2,t)
    end
  elseif stage==2 then
    local qLArmTarget = arm_planner:playback_trajectory(t)
    if qLArmTarget then movearm.setArmJoints(qLArmTarget,qRArm,dt)
    else 
      stage = stage+1 
    end
  elseif stage==3 then
    gripL,doneL = util.approachTol(gripL,1,2,dt)
    Body.set_lgrip_percent(gripL*0.8)
    Body.set_rgrip_percent(gripR*0.8)    
    if doneL then
      stage=stage+1
    end
  end
    

--[[
  elseif stage==4 then
    local trRArmTarget = movearm.getToolPosition(tool_pos_right0,0.08,0)    
    local trRArmApproach, doneL = util.approachTolTransform(trRArm, trRArmTarget, dpArmMax, dt)    
    local ret = movearm.setArmToPosition(trLArm,trRArmApproach,dt,    
    45*math.pi/180,-45*math.pi/180)
    if ret==1 and doneL then stage=stage+1; end
  elseif stage==5 then
    local trRArmTarget = movearm.getToolPosition(tool_pos_right,0.08,0)    
    local trRArmApproach, doneR = util.approachTolTransform(trRArm, trRArmTarget, dpArmMax, dt)    
    local ret = movearm.setArmToPosition(trLArm,trRArmApproach,dt,    
    45*math.pi/180,-45*math.pi/180)
    if ret==1 and doneR then stage=stage+1; end
  elseif stage==6 then
    local trRArmTarget = movearm.getToolPosition(tool_pos_right,0,0)    
    local trRArmApproach, doneR = util.approachTolTransform(trRArm, trRArmTarget, dpArmMax, dt)    
    local ret = movearm.setArmToPosition(trLArm,trRArmApproach,dt,    
    45*math.pi/180,-45*math.pi/180)
    if ret==1 and doneR then 
    
      stage=stage+1; 
    end
  elseif stage==7 then
    local trLArmTarget = movearm.getToolPosition(tool_pos_left2,0,1)    
    local trRArmTarget = movearm.getToolPosition(tool_pos_right2,0,0)    
    local trLArmApproach, doneL = util.approachTolTransform(trLArm, trLArmTarget, dpArmMax, dt)
    local trRArmApproach, doneR = util.approachTolTransform(trRArm, trRArmTarget, dpArmMax, dt)    
    local ret = movearm.setArmToPosition(trLArm,trRArmApproach,dt,    
      45*math.pi/180,-45*math.pi/180)
  end

  --]]
end

function state.exit()  
  Body.set_lgrip_percent(0)
  Body.set_rgrip_percent(0)
  print(state._NAME..' Exit' )
end

return state