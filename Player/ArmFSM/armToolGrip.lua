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

local debugdata

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

  qLArm0 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, 0,0*Body.DEG_TO_RAD, -45*Body.DEG_TO_RAD})
  qRArm0 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, 0,0*Body.DEG_TO_RAD, 45*Body.DEG_TO_RAD})
  
  local trLArm0 = Body.get_forward_larm(qLArm0)
  local trRArm0 = Body.get_forward_rarm(qRArm0)  
 
  --tool_pos_left = hcm.get_tool_pos()
  tool_pos_left=vector.new({0.55,0.02,0.05})  
  tool_pos_left1=tool_pos_left + vector.new({0,0,0.05})
  tool_pos_left2=vector.new({0.35,0.05,tool_pos_left1[3]})  
  tool_pos_left3=vector.new({0.30,0.10,-0.10})  

--  trLArmTarget0 = trLArm
  trLArmTarget1 = movearm.getToolPosition(tool_pos_left,0.08,1)    
  trLArmTarget2 = movearm.getToolPosition(tool_pos_left,0,1)    

  trLArmTarget3 = movearm.getToolPosition(tool_pos_left1,0,1)   --lift up 
  trLArmTarget4 = movearm.getToolPosition(tool_pos_left2,0,1)    
  trLArmTarget5 = movearm.getToolPosition(tool_pos_left3,0,1)    

  print("Planning LArmPlan1")
  LArmPlan1,qLArm1 = arm_planner:plan_arm(qLArm0, trLArmTarget1, 1)  
  print("Planning LArmPlan2")
  LArmPlan2,qLArm1 = arm_planner:plan_arm(qLArm1, trLArmTarget2, 1)
  print("Planning LArmPlan3")
  LArmPlan3,qLArm1 = arm_planner:plan_arm(qLArm1, trLArmTarget3, 1)
  print("Planning LArmPlan4")
  LArmPlan4,qLArm1 = arm_planner:plan_arm(qLArm1, trLArmTarget4, 1)
  print("Planning LArmPlan5")
  LArmPlan5,qLArm1 = arm_planner:plan_arm(qLArm1, trLArmTarget5, 1)

  
  
  --Test two arm planning
  print("Testing two-arm planning")
  arm_planner:set_hand_mass(0,0)
  LAP1, RAP1, uTP1, qLArm1, qRArm1, uTorsoComp1 = arm_planner:plan_double_arm(qLArm0,qRArm0,trLArmTarget1,trRArm0, {0,0})
  LAP2, RAP2, uTP2, qLArm1, qRArm1, uTorsoComp1 = arm_planner:plan_double_arm(qLArm1,qRArm1,trLArmTarget2,trRArm0, uTorsoComp1)

  arm_planner:set_hand_mass(2.0,0)
  LAP3, RAP3, uTP3, qLArm1, qRArm1, uTorsoComp1 = arm_planner:plan_double_arm(qLArm1,qRArm1,trLArmTarget3,trRArm0, uTorsoComp1)
  LAP4, RAP4, uTP4, qLArm1, qRArm1, uTorsoComp1 = arm_planner:plan_double_arm(qLArm1,qRArm1,trLArmTarget4,trRArm0, uTorsoComp1)





  stage = 0;  
  debugdata=''
end

local gripL, gripR = 0,0

local qJointVelTool = 
  {10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,
   30*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,30*Body.DEG_TO_RAD,}

function state.update()
  --  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end
  qLArm = Body.get_larm_command_position()
  qRArm = Body.get_rarm_command_position()  

  local trLArm = Body.get_forward_larm(qLArm)


  debugdata=debugdata..
    string.format("%f,  %f,%f,%f,%f,%f,%f,%f,  %f,%f,%f,%f,%f,%f\n",
    t-t_entry, 
    qLArm[1],
    qLArm[2],
    qLArm[3],
    qLArm[4],
    qLArm[5],
    qLArm[6],
    qLArm[7],

    trLArm[1],
    trLArm[2],
    trLArm[3],
    trLArm[4],
    trLArm[5],
    trLArm[6]
    )


--  if stage==2 then return end

  if stage==0 then --Rotate wrist
    ret = movearm.setArmJoints(qLArm0, qRArm0 ,dt, qJointVelTool)
    if ret==1 then 
      arm_planner:init_trajectory(LArmPlan1,t)
      stage=stage+1 
    end
  elseif stage==1 then
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
      arm_planner:init_trajectory(LArmPlan3,t)
    end
  elseif stage==4 then
    local qLArmTarget = arm_planner:playback_trajectory(t)
    if qLArmTarget then movearm.setArmJoints(qLArmTarget,qRArm,dt)
    else 
      stage = stage+1  
      arm_planner:init_trajectory(LArmPlan4,t)
    end
  elseif stage==5 then
    local qLArmTarget = arm_planner:playback_trajectory(t)
    if qLArmTarget then movearm.setArmJoints(qLArmTarget,qRArm,dt)
    else 
      stage = stage+1  
      arm_planner:init_trajectory(LArmPlan5,t)
    end
  elseif stage==6 then
    local qLArmTarget = arm_planner:playback_trajectory(t)
    if qLArmTarget then movearm.setArmJoints(qLArmTarget,qRArm,dt)
    else 
      stage = stage+1        
    end
  end
end

function state.exit()  


  local savefile = string.format("Log/debugdata_%s",os.date());
  local debugfile=assert(io.open(savefile,"w")); 
  debugfile:write(debugdata);
  debugfile:flush();
  debugfile:close();  

  Body.set_lgrip_percent(0)
  Body.set_rgrip_percent(0)
  print(state._NAME..' Exit' )
end

return state