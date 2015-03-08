local state = {}
state._NAME = ...
require'hcm'
require'mcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()

--Initial hand angle
local lhand_rpy = Config.armfsm.toolgrip.lhand_rpy
local rhand_rpy = Config.armfsm.toolgrip.rhand_rpy
local stage
local debugdata

local function get_tool_tr(tooloffset)
  local handrpy = rhand_rpy
  local tool_model = hcm.get_tool_model()
  local hand_pos = vector.slice(tool_model,1,3) + vector.new({tooloffset[1],tooloffset[2],tooloffset[3]})  
  local tool_tr = {hand_pos[1],hand_pos[2],hand_pos[3], handrpy[1],handrpy[2],handrpy[3] + tool_model[4]}
  print("hand transform:",util.print_transform(tool_tr))                    
  return tool_tr
end

local function get_hand_tr(pos) return {pos[1],pos[2],pos[3], unpack(rhand_rpy)} end

local function check_override()
  local override = hcm.get_state_override()
  for i=1,7 do if override[i]~=0 then return true end end
  return false
end

local function update_override()
  local override = hcm.get_state_override()
  local tool_model = hcm.get_tool_model()

  tool_model[1],tool_model[2],tool_model[3], tool_model[4] = 
  tool_model[1] + override[1],
  tool_model[2] + override[2],
  tool_model[3] + override[3],
  tool_model[4] + override[6]*5*DEG_TO_RAD, --yaw
--print("upd: old model",unpack(hcm.get_tool_model() ))

--SJ: this is weird, we need to read once to update the shm
  hcm.get_tool_model()
  hcm.set_tool_model(tool_model)
  print( util.color('Tool model:','yellow'), string.format("%.2f %.2f %.2f / %.1f",
        tool_model[1],tool_model[2],tool_model[3],tool_model[4]*180/math.pi ))
  hcm.set_state_proceed(0)
end

local function revert_override()
  print("revert")
  local override = hcm.get_state_override()
  local tool_model = hcm.get_tool_model()

  tool_model[1],tool_model[2],tool_model[3], tool_model[4] = 
  tool_model[1] - override[1],
  tool_model[2] - override[2],
  tool_model[3] - override[3],
  tool_model[4] - override[6]*5*DEG_TO_RAD, --yaw

--SJ: this is weird, we need to read once to update the shm
  hcm.get_tool_model()

  hcm.set_tool_model(tool_model)
  print( util.color('Tool model:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f",
        tool_model[1],tool_model[2],tool_model[3],
        tool_model[4]*180/math.pi ))
  hcm.set_state_proceed(0)
  hcm.set_state_override({0,0,0,0,0,0,0})  

end

local function confirm_override() hcm.set_state_override({0,0,0,0,0,0,0}) end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_rhandoffset(Config.arm.handoffset.gripper)
  hcm.set_tool_model(Config.armfsm.toolgrip.default_model)

  arm_planner:set_hand_mass(0,0)
  arm_planner:set_shoulder_yaw_target(nil, nil)   
  mcm.set_arm_endpoint_compensation({0,1}) -- compensate for torso movement for only right hand

  stage="start"
  hcm.set_state_proceed(1)

  debugdata=''   
  hcm.set_state_tstartactual(unix.time())
  hcm.set_state_tstartrobot(Body.get_time())
end

function state.update()
  --  print(state._NAME..' Update' )
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t   -- Save this at the last update time

  local cur_cond = arm_planner:load_boundary_condition()
  local trLArm = Body.get_forward_larm(cur_cond[1])
  local trRArm = Body.get_forward_rarm(cur_cond[2])  
  
----------------------------------------------------------
--Forward motions
----------------------------------------------------------

  if stage=="start" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        local arm_seq = util.shallow_copy(Config.armfsm.toolgrip.arminit)
        table.insert(arm_seq,{'move',nil, get_tool_tr({0,0,0})})
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "arminit" end  
        hcm.set_state_proceed(0) --stop here
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_shoulder_yaw_target(nil, Config.arm.ShoulderYaw0[2])   
        local arm_seq = {{'move0',nil, Config.arm.trRArm0}}
        if arm_planner:plan_arm_sequence(arm_seq) then stage = "armbacktoinitpos" end
      end  
    end        
  elseif stage=="arminit" then --Grip the object   
    if arm_planner:play_arm_sequence(t) then    
      if hcm.get_state_proceed()==1 then        
        local arm_seq = {{'move',nil,get_tool_tr({0,0,0})}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "lift" end
        
      elseif hcm.get_state_proceed()==-1 then 
        if arm_planner:plan_arm_sequence(Config.armfsm.toolgrip.armuninit) then stage = "start" end                  
     
      elseif check_override() then --Model modification

        print("Current tr:",util.print_transform(trRArm))
          

        print("arminit")
        update_override()                
        local arm_seq = {{'move',nil,get_tool_tr({0,0,0})}}        
        if arm_planner:plan_arm_sequence(arm_seq) then 
          stage = "arminit" 
          confirm_override()
        else revert_override() end
      end
    end
    
  elseif stage=="lift" then
    if arm_planner:play_arm_sequence(t) then    
      if hcm.get_state_proceed()==1 then        
        print("trRArm:",util.print_transform(trRArm))        
        if arm_planner:plan_arm_sequence2(Config.armfsm.toolgrip.armpull) then stage = "liftpull" end

      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_hand_mass(0,1)   
        local arm_seq = {{'move',nil,get_tool_tr({0,0,0})}}   
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "arminit" end
      
      elseif check_override() then --Model modification
        print("LIFT")
        update_override()        
        local arm_seq = {{'move',nil,get_tool_tr({0,0,0})}}   
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "lift" 
        confirm_override()
        else revert_override() end
      end
    end

  elseif stage=="liftpull" then --Move arm back to holding position
    if arm_planner:play_arm_sequence(t) then    
      stage = "pulldone"
      print("SEQUENCE DONE")
      return"hold"      
    end      
  elseif stage=="armbacktoinitpos" then 
    if arm_planner:play_arm_sequence(t) then return "done" end
  end

  
end

function state.exit()  
  hcm.set_state_success(1) --Report success
  --Store boundary conditions for future state
  --arm_planner:save_boundary_condition(current_arm_endcond)
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
