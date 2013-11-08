local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()

local gripL, gripR = 0,0
local stage = 1
local plan_failed = false

local arm_plan1, end_arm_sequence1

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  local init_cond = arm_planner:load_boundary_condition()
  local trLArm0 = Body.get_forward_larm(init_cond[1])
  local trRArm0 = Body.get_forward_rarm(init_cond[2]) 
    
  local tool_pos_left0=vector.new({0.20,0.0,-0.10})  
  local tool_pos_left1=vector.new({0.25,0.0,0.0})  

  local tool_pos_left2=vector.new({0.40,0.0,0.0})  
  local tool_pos_left3=vector.new({0.40,0.17,0.0})  
  local tool_pos_left4=vector.new({0.40,0,0.20})  
  local tool_pos_left5=vector.new({0.40,-0.13,0.0})  
  local tool_pos_left6=vector.new({0.40,0,0.0})  


  local trLArmTarget1 = movearm.getToolPosition(tool_pos_left0,0,1)    
  local trLArmTarget2 = movearm.getToolPosition(tool_pos_left1,0,1)    
  local trLArmTarget3 = movearm.getToolPosition(tool_pos_left2,0,1)    
  local trLArmTarget4 = movearm.getToolPosition(tool_pos_left3,0,1)    
  local trLArmTarget5 = movearm.getToolPosition(tool_pos_left4,0,1)    
  local trLArmTarget6 = movearm.getToolPosition(tool_pos_left5,0,1)    
  local trLArmTarget7 = movearm.getToolPosition(tool_pos_left6,0,1)    

  local arm_sequence1 = {
    init=init_cond,
    mass={3,0},
    armseq={
      {trLArmTarget1, trRArm0},
      {trLArmTarget2, trRArm0},
      {trLArmTarget3, trRArm0},
      {trLArmTarget4, trRArm0},
      {trLArmTarget5, trRArm0},      
      {trLArmTarget6, trRArm0},      
      {trLArmTarget7, trRArm0},            
    }
  }
  arm_plan1, arm_end1 = arm_planner:plan_arm_sequence(arm_sequence1)

  if not arm_plan1 then plan_failed = true end
  stage = 1  
  arm_planner:init_arm_sequence(arm_plan1,t_entry)  
end

function state.update()
  --  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  if plan_failed then return "planfail" end
  if stage==1 then
    if arm_planner:play_arm_sequence(t) then    
      stage = stage+1
    end
  end
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state