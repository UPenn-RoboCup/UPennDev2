local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()


local qLArm0,qRArm0, trLArm0, trRArm0

--Initial hand angle
local lhand_rpy0 = {0,0*Body.DEG_TO_RAD, -45*Body.DEG_TO_RAD}
local rhand_rpy0 = {0,0*Body.DEG_TO_RAD, 45*Body.DEG_TO_RAD}
local stage

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
  
  local init_cond = arm_planner:load_boundary_condition()
  trLArm0 = Body.get_forward_larm(init_cond[1])
  trRArm0 = Body.get_forward_rarm(init_cond[2]) 

    
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



  if not arm_plan1 then plan_failed = true end
  
  stage = "drillout"

end

function state.update()
  --  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  
  if stage=="drillout" then
    if hcm.get_state_proceed()==1 then 
      local trLArmTarget1 = {0.30,0,-0.10, unpack(lhand_rpy0)}
      local trLArmTarget2 = {0.30,0,    0, unpack(lhand_rpy0)}
      local arm_seq = {      
        mass={3,0},
        armseq={
          {trLArmTarget1, trRArm0},
          {trLArmTarget2, trRArm0},
        }
      }
      if arm_planner:plan_arm_sequence(arm_seq) then stage="drilloutmove" end
    end
  elseif stage=="drilloutmove" then
    if arm_planner:play_arm_sequence(t) then stage = "drilloutwait" end

  elseif stage=="drilloutwait" then
    if hcm.get_state_proceed()==1 then 
      local cutpos1 = hcm.get_tool_cutpos1()
      local trLArmTarget1 = {0.30,cutpos1[2],cutpos1[3],unpack(lhand_rpy0)}
      local arm_seq = {      
        mass={3,0},
        armseq={
          {trLArmTarget1, trRArm0},        
        }
      }
      if arm_planner:plan_arm_sequence(arm_seq) then stage = "drillposition" end      
    elseif hcm.get_state_proceed()==-1 then       
      local trLArmTarget1 = {0.30,0,-0.10, unpack(lhand_rpy0)}
      local trLArmTarget2 = {0.20,0,-0.10, unpack(lhand_rpy0)}
      local arm_seq = {      
        mass={3,0},
        armseq={
          {trLArmTarget1, trRArm0},
          {trLArmTarget2, trRArm0},
        }
      }
      if arm_planner:plan_arm_sequence(arm_seq) then stage = "undrillout" end
    end
  elseif stage=="drillposition" then
    if arm_planner:play_arm_sequence(t) then stage = "drillpositionwait" end

  elseif stage=="drillpositionwait" then
    if hcm.get_state_proceed()==1 then 
      local cutpos1 = hcm.get_tool_cutpos1()
      local cutpos2 = hcm.get_tool_cutpos2()
      local trLArmTarget1 = {cutpos1[1],cutpos1[2],cutpos1[3],unpack(lhand_rpy0)}
      local trLArmTarget2 = {cutpos2[1],cutpos2[2],cutpos2[3],unpack(lhand_rpy0)}
      local trLArmTarget3 = {0.30,cutpos2[2],cutpos2[3],unpack(lhand_rpy0)}
      local arm_seq = {      
        mass={3,0},
        armseq={
          {trLArmTarget1, trRArm0},        
          {trLArmTarget2, trRArm0},        
          {trLArmTarget3, trRArm0},    
        }
      }
      if arm_planner:plan_arm_sequence(arm_seq) then stage = "drillcut" end
    elseif hcm.get_state_proceed()==-1 then 
      local trLArmTarget1 = {0.30,0,    0, unpack(lhand_rpy0)}      
      local arm_seq = {      
        mass={3,0},
        armseq={
          {trLArmTarget1, trRArm0},
          {trLArmTarget2, trRArm0},
        }
      }
      if arm_planner:plan_arm_sequence(arm_seq) then stage = "undrillposition" end
    end
  elseif stage=="drillcut" then
    if arm_planner:play_arm_sequence(t) then stage = "drillpositionwait" end
  elseif stage=="undrillposition" then
    if arm_planner:play_arm_sequence(t) then stage = "drilloutwait" end
  elseif stage=="undrillout" then
    if arm_planner:play_arm_sequence(t) then return "done" end
  end
  
  hcm.set_state_proceed(0)
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state