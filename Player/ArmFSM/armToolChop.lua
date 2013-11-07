local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'

local dqArmMax = Config.arm.slow_limit
local dpArmMax = Config.arm.linear_slow_limit
local stage = 1;

local qLArm, qRArm, qLArm0, qRArm0, trLArm, trRArm, trTarget
local tool_pos,tool_pos_target

local qLArmTarget0, qRArmTarget0
local trLArmTarget,trRArmTarget

local libArmPlan = require 'libArmPlan'
arm_planner = libArmPlan.new_planner()

local gripL, gripR = 0,0

local debugdata

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  
  qLArm = Body.get_larm_command_position()
  qRArm = Body.get_rarm_command_position()

  tool_pos_left1=tool_pos_left + vector.new({0,0,0.05})
  tool_pos_left2=vector.new({0.35,0.05,tool_pos_left1[3]})  

  tool_pos_left0=vector.new({0.20,0.0,-0.10})  
  tool_pos_left1=vector.new({0.25,0.0,0.0})  
  tool_pos_left2=vector.new({0.40,0.0,0.0})  
  tool_pos_left3=vector.new({0.40,0.15,0.0})  
  tool_pos_left4=vector.new({0.40,-0.15,0.0})  
  



  trLArmTarget1 = movearm.getToolPosition(tool_pos_left0,0,1)    
  trLArmTarget2 = movearm.getToolPosition(tool_pos_left1,0,1)    
  trLArmTarget3 = movearm.getToolPosition(tool_pos_left2,0,1)    
  trLArmTarget4 = movearm.getToolPosition(tool_pos_left3,0,1)    
  trLArmTarget5 = movearm.getToolPosition(tool_pos_left4,0,1)    

  

  local qLArm0 = mcm.get_arm_qlarm()
  local qRArm0 = mcm.get_arm_qrarm()        

  local trLArm0 = Body.get_forward_larm(qLArm0)
  local trRArm0 = Body.get_forward_rarm(qRArm0) 

  local qLArmComp0 = mcm.get_arm_qlarmcomp()
  local qRArmComp0 = mcm.get_arm_qrarmcomp()
  local uTorsoComp0 = mcm.get_stance_uTorsoComp()
  
  LAP1, RAP1, uTP1, qLArm1, qRArm1, qLArmComp1, qRArmComp1, uTorsoComp1 = 
    arm_planner:plan_double_arm(qLArm0,qRArm0,qLArmComp0, qRArmComp0, trLArmTarget1,trRArm0, uTorsoComp0)

  LAP2, RAP2, uTP2, qLArm1, qRArm1, qLArmComp1, qRArmComp1, uTorsoComp1 = 
    arm_planner:plan_double_arm(qLArm1,qRArm1,qLArmComp1, qRArmComp1, trLArmTarget2,trRArm0, uTorsoComp1)

  LAP3, RAP3, uTP3, qLArm1, qRArm1, qLArmComp1, qRArmComp1, uTorsoComp1 = 
    arm_planner:plan_double_arm(qLArm1,qRArm1,qLArmComp1, qRArmComp1, trLArmTarget3,trRArm0, uTorsoComp1)

  LAP4, RAP4, uTP4, qLArm1, qRArm1, qLArmComp1, qRArmComp1, uTorsoComp1 = 
    arm_planner:plan_double_arm(qLArm1,qRArm1,qLArmComp1, qRArmComp1, trLArmTarget4,trRArm0, uTorsoComp1)    

  LAP5, RAP5, uTP5, qLArm1, qRArm1, qLArmComp1, qRArmComp1, uTorsoComp1 = 
    arm_planner:plan_double_arm(qLArm1,qRArm1,qLArmComp1, qRArmComp1, trLArmTarget5,trRArm0, uTorsoComp1)        



  stage = 1  
  debugdata=''


  arm_planner:init_trajectory_double(LAP1, RAP1, uTP1 ,t_entry)
end



function state.update()
  --  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  if not LAP3 then --Plan failed. escape
    return "planfail"
  end

  --if t-t_entry > timeout then return'timeout' end
  qLArm = Body.get_larm_command_position()
  qRArm = Body.get_rarm_command_position()  

  local trLArm = Body.get_forward_larm(qLArm)

  if stage==1 then
    local qLArmTarget, qRArmTarget = arm_planner:playback_trajectory_double(t)
    if not qLArmTarget then       
      stage = stage+1       
      arm_planner:init_trajectory_double(LAP2, RAP2, uTP2 ,t)
    end  
  elseif stage==2 then
    local qLArmTarget, qRArmTarget = arm_planner:playback_trajectory_double(t)
    if not qLArmTarget then       
      stage = stage+1       
      arm_planner:init_trajectory_double(LAP3, RAP3, uTP3 ,t)
    end
  elseif stage==3 then
    local qLArmTarget, qRArmTarget = arm_planner:playback_trajectory_double(t)
    if not qLArmTarget then       
      stage = stage+1       
      arm_planner:init_trajectory_double(LAP4, RAP4, uTP4 ,t)
    end
  elseif stage==4 then
    local qLArmTarget, qRArmTarget = arm_planner:playback_trajectory_double(t)
    if not qLArmTarget then       
      stage = stage+1       
      arm_planner:init_trajectory_double(LAP5, RAP5, uTP5 ,t)
    end
  elseif stage==5 then
    local qLArmTarget, qRArmTarget = arm_planner:playback_trajectory_double(t)
    if not qLArmTarget then       
      stage = stage+1       
    end
  end
end

function state.exit()  

--[[
  local savefile = string.format("Log/debugdata_%s",os.date());
  local debugfile=assert(io.open(savefile,"w")); 
  debugfile:write(debugdata);
  debugfile:flush();
  debugfile:close();  
--]]

  print(state._NAME..' Exit' )
end

return state