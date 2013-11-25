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


local function get_cutpos1_tr(tooloffset)
  tooloffset = tooloffset or {0,0,0}
  local handrpy = rhand_rpy0
  local cutpos = hcm.get_tool_cutpos1()
  local cutpos_tr = {
    cutpos[1] + tooloffset[1],
    cutpos[2] + tooloffset[2],
    cutpos[3] + tooloffset[3],
    handrpy[1],handrpy[2],handrpy[3] + cutpos[4]}
  return cutpos_tr
end

local function get_cutpos2_tr(tooloffset)
  tooloffset = tooloffset or {0,0,0}
  local handrpy = rhand_rpy0
  local cutpos = hcm.get_tool_cutpos2()
  local cutpos_tr = {
    cutpos[1] + tooloffset[1],
    cutpos[2] + tooloffset[2],
    cutpos[3] + tooloffset[3],
    handrpy[1],handrpy[2],handrpy[3] + cutpos[4]}
  return cutpos_tr
end

local function get_hand_tr(pos)
  return {pos[1],pos[2],pos[3], unpack(rhand_rpy0)}
end

local function update_model()
  local trRArmTarget = hcm.get_hands_right_tr_target()
  local trRArm = hcm.get_hands_right_tr()
  local tool_cutpos1 = hcm.get_tool_cutpos1()
  local tool_cutpos2 = hcm.get_tool_cutpos2()

  tool_cutpos1[1],tool_cutpos1[2],tool_cutpos1[3],tool_cutpos1[4]=

    tool_cutpos1[1] + trRArmTarget[1] - trRArm[1],
    tool_cutpos1[2] + trRArmTarget[2] - trRArm[2],
    tool_cutpos1[3] + trRArmTarget[3] - trRArm[3],
    tool_cutpos1[4] + util.mod_angle(trRArmTarget[6] - trRArm[6])

  hcm.set_tool_cutpos1(tool_cutpos1)
end




local stage
local cut_no

local function update_cutpos()
  cut_no = (cut_no+1)%3
  if cut_no==0 then
    hcm.set_tool_cutpos1({0.40,0.22, 0.2892, 0})
    hcm.set_tool_cutpos2({0.40,0.22, -0.1172,   0})    
  elseif cut_no==1 then
    hcm.set_tool_cutpos1({0.40,0.22,-0.1172,   0})    
    hcm.set_tool_cutpos2({0.40,-0.38,-0.1172,   0})
  else
    hcm.set_tool_cutpos1({0.40,-0.38,-0.1172, 0})
    hcm.set_tool_cutpos2({0.40,0.22,0.2892, 0})
  end
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  qLArm0 = qLArm
  qRArm0 = qRArm
  
  --arm_planner:set_shoulder_yaw_target(nil,qRArm[3]) --Lock right shoulder yaw
  arm_planner:set_shoulder_yaw_target(qLArm[3],nil) --Lock right shoulder yaw
  local init_cond = arm_planner:load_boundary_condition()
  arm_planner:set_hand_mass(0,2)
  trLArm0 = Body.get_forward_larm(init_cond[1])
  trRArm0 = Body.get_forward_rarm(init_cond[2]) 
  
  stage = "drillout"
  cut_no=0
  update_cutpos()
--  hcm.set_state_proceed(1)
end

function state.update()
  --  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t

  local cur_cond = arm_planner:load_boundary_condition()
  local trLArm = Body.get_forward_larm(cur_cond[1])
  local trRArm = Body.get_forward_rarm(cur_cond[2])  
  
  if stage=="drillout" then
    if hcm.get_state_proceed()==1 then 
      local trRArmTarget1 = get_hand_tr(Config.armfsm.toolchop.arminit[2])
      local trRArmTarget2 = get_hand_tr(Config.armfsm.toolchop.arminit[3])
      local arm_seq = {{'move',nil,trRArmTarget1},{'move',nil,trRArmTarget2}}
      if arm_planner:plan_arm_sequence2(arm_seq) then stage="drilloutmove" end
    end
  elseif stage=="drilloutmove" then
    if arm_planner:play_arm_sequence(t) then 
      stage = "drilloutwait" 
      --Torso Y now should follow arm movement
      arm_planner:start_torso_track(0)--right hand      
    end
  elseif stage=="drilloutwait" then
    if hcm.get_state_proceed()==1 then       
      local trRArmTarget1 = get_cutpos1_tr(Config.armfsm.toolchop.drill_clearance)      
      local arm_seq = {{'move',nil, trRArmTarget1}}
      if arm_planner:plan_arm_sequence2(arm_seq) then stage = "drillposition" end      
    elseif hcm.get_state_proceed()==-1 then       
      local trRArmTarget1 = get_hand_tr(Config.armfsm.toolchop.arminit[2])
      local trRArmTarget2 = get_hand_tr(Config.armfsm.toolchop.arminit[1])
      local arm_seq = {{'move',nil, trRArmTarget1},{'move',nil, trRArmTarget2}}
      if arm_planner:plan_arm_sequence2(arm_seq) then stage = "undrillout" end
    end
  elseif stage=="drillposition" then
    if arm_planner:play_arm_sequence(t) then stage = "drillpositionwait" end
  elseif stage=="drillpositionwait" then
    if hcm.get_state_proceed()==1 then 
      local trRArmTarget1 = get_cutpos1_tr(Config.armfsm.toolchop.drill_clearance)
      local trRArmTarget2 = get_cutpos1_tr()
      local trRArmTarget3 = get_cutpos2_tr()
      local trRArmTarget4 = get_cutpos2_tr(Config.armfsm.toolchop.drill_clearance)
      local arm_seq = {
        {'move',nil,trRArmTarget1},{'move',nil,trRArmTarget2},
        {'move',nil,trRArmTarget3},{'move',nil,trRArmTarget4}}
      if arm_planner:plan_arm_sequence2(arm_seq) then stage = "drillcut" end
    elseif hcm.get_state_proceed()==-1 then 
      local trRArmTarget1 = get_hand_tr(Config.armfsm.toolchop.arminit[3])
      local arm_seq = {{'move',nil,trRArmTarget1}}
      if arm_planner:plan_arm_sequence2(arm_seq) then stage = "undrillposition" end
    elseif hcm.get_state_proceed() == 2 then --Model modification
      update_model()        
      local trRArmTarget1 = get_cutpos1_tr(Config.armfsm.toolchop.drill_clearance)      
      local arm_seq = {{'move',nil, trRArmTarget1}}
      if arm_planner:plan_arm_sequence2(arm_seq) then stage = "drillposition" end      
    end
  elseif stage=="drillcut" then
    if arm_planner:play_arm_sequence(t) then 
      update_cutpos()
      stage = "drillpositionwait" 
    end
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