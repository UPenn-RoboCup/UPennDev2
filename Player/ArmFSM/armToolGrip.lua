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

  --open gripper
  Body.set_lgrip_percent(0)
  Body.set_rgrip_percent(0)

  qLArm = Body.get_larm_command_position()
  qRArm = Body.get_rarm_command_position()

  --Initial hand angle

  local lhand_rpy0 = {0,0*Body.DEG_TO_RAD, -45*Body.DEG_TO_RAD}
  local rhand_rpy0 = {0,0*Body.DEG_TO_RAD, 45*Body.DEG_TO_RAD}

  qLArm0 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})
  qRArm0 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  
  local trLArm0 = Body.get_forward_larm(qLArm0)
  local trRArm0 = Body.get_forward_rarm(qRArm0)  
 
  --tool_pos_left = hcm.get_tool_pos()
  --tool_pos_left=vector.new({0.55,0.02,0.05})  

  --with 0 bodytilt, robot has slightly shorter reach
  tool_pos_left=vector.new({0.51,0.02,0.05})  
  tool_yaw = 0 --TODO




  tool_pos_left1=tool_pos_left + vector.new({0,0,0.05})
  tool_pos_left2=vector.new({0.35,0.05,tool_pos_left1[3]})  
  tool_pos_left3=vector.new({0.20,0.0,-0.10})  

  trLArmTarget1 = movearm.getToolPosition(tool_pos_left,0.08,1)    
  trLArmTarget2 = movearm.getToolPosition(tool_pos_left,0,1)    
  trLArmTarget3 = movearm.getToolPosition(tool_pos_left1,0,1)   --lift up 
  trLArmTarget4 = movearm.getToolPosition(tool_pos_left2,0,1)    
  trLArmTarget5 = movearm.getToolPosition(tool_pos_left3,0,1)    


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
  arm_plan1, end_arm_sequence1 = arm_planner:plan_arm_sequence(arm_sequence1)

  local arm_sequence2 = {
    init=end_arm_sequence1,
    mass={3,0},
    armseq={
      {trLArmTarget3, trRArm0},
      {trLArmTarget4, trRArm0},
      {trLArmTarget5, trRArm0},
    }
  }
  arm_plan2,end_arm_sequence2 = arm_planner:plan_arm_sequence(arm_sequence2)

  stage = 0;  
  debugdata=''
  uTorsoCompTarget = {0,0}


  mcm.set_arm_qlarm(end_arm_sequence2[1])
  mcm.set_arm_qrarm(end_arm_sequence2[2])        
  mcm.set_arm_qlarmcomp(end_arm_sequence2[3])
  mcm.set_arm_qrarmcomp(end_arm_sequence2[4])


end



function state.update()
  --  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  if not arm_plan2 then --Plan failed. escape
    return "planfail"
  end

  if stage==0 then --Rotate wrist
    ret = movearm.setArmJoints(qLArm0, qRArm0 ,dt, Config.arm.joint_init_limit)
    if ret==1 then     
      arm_planner:init_arm_sequence(arm_plan1,t)
      stage=stage+1 
    end
  elseif stage==1 then
    if arm_planner:play_arm_sequence(t) then    
      stage = stage+1             
    end
  elseif stage==2 then    
    gripL,doneL = util.approachTol(gripL,1,2,dt)
    Body.set_lgrip_percent(gripL*0.8)
    Body.set_rgrip_percent(gripR*0.8)    
    if doneL then
      stage=stage+1
      arm_planner:init_arm_sequence(arm_plan2,t)
    end
  elseif stage==3 then
    if arm_planner:play_arm_sequence(t) then    
      print("SEQUENCE DONE")
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