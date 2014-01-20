local state = {}
state._NAME = ...
require'hcm'
require'mcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()

local qLArm0,qRArm0, trLArm0, trRArm0, trLArm1, trARArm1

--Initial hand angle
local lhand_rpy = Config.armfsm.firesuppress.lhand_rpy
local rhand_rpy = Config.armfsm.firesuppress.rhand_rpy


local gripL, gripR = 1,1
local stage
local debugdata

local function get_fire_tr(fireoffset)

  local handrpy = rhand_rpy
  local fire_model = hcm.get_fire_model()  -- xyz, pitch, yaw
  local hand_pos = vector.slice(fire_model,1,3) + 
    vector.new({fireoffset[1],fireoffset[2],fireoffset[3]})  
  local fire_tr = {hand_pos[1],hand_pos[2],hand_pos[3],
                    handrpy[1],handrpy[2]+fire_model[4], handrpy[3]+fire_model[5]}

  print("hand transform:",arm_planner.print_transform(fire_tr))                    
  return fire_tr
end

local function get_hand_tr(pos)
  return {pos[1],pos[2],pos[3], unpack(rhand_rpy)}
end

local function check_override()
  local override = hcm.get_state_override()
  for i=1,7 do
    if override[i]~=0 then return true end
  end
  return false
end

local function check_override_rotate()
  local override = hcm.get_state_override()
  for i=4,6 do
    if override[i]~=0 then return true end
  end
  return false
end

local function update_override()
  local override = hcm.get_state_override()
  local fire_model = hcm.get_fire_model()

	-- Update target tranform
  fire_model[1],fire_model[2],fire_model[3], fire_model[4], fire_model[5] = 
  fire_model[1] + override[1],
  fire_model[2] + override[2],
  fire_model[3] + override[3],
  fire_model[4] + override[5]*Body.DEG_TO_RAD, --pitch
  fire_model[5] + override[6]*Body.DEG_TO_RAD, --yaw

--SJ: this is weird, we need to read once to update the shm
  hcm.get_fire_model()
  hcm.set_fire_model(fire_model)

  print( util.color('Fire model:','yellow'), 
        string.format("%.2f %.2f %.2f / %.1f %.1f",
        fire_model[1],fire_model[2],fire_model[3],
        fire_model[4]*Body.RAD_TO_DEG,
        fire_model[5]*Body.RAD_TO_DEG) 
       )
  
	hcm.set_state_proceed(0)
end

local function clear_override()
  hcm.set_state_override({0,0,0,0,0,0,0})
end

local function revert_override()
  print("revert")
  local override = hcm.get_state_override()
  local fire_model = hcm.get_fire_model()

  fire_model[1],fire_model[2],fire_model[3], fire_model[4], fire_model[5] = 
  fire_model[1] - override[1],
  fire_model[2] - override[2],
  fire_model[3] - override[3],
  fire_model[4] - override[5]*Body.DEG_TO_RAD, --pitch
  fire_model[5] - override[6]*Body.DEG_TO_RAD, --yaw

--SJ: this is weird, we need to read once to update the shm
  hcm.get_fire_model()
  hcm.set_fire_model(fire_model)
	
  print( util.color('Fire model:','yellow'), 
        string.format("%.2f %.2f %.2f / %.1f %.1f",
        fire_model[1],fire_model[2],fire_model[3],
        fire_model[4]*Body.RAD_TO_DEG,
        fire_model[5]*Body.RAD_TO_DEG)
       )
				
  hcm.set_state_proceed(0)
  clear_override()  

end


function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_rhandoffset(Config.arm.handoffset.gripper)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

	-- Initial configuration
  qLArm0 = qLArm
  qRArm0 = qRArm
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

  --Initial arm joint angles after rotating wrist
  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm0, Config.armfsm.firesuppress.larm[1])
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm0, Config.armfsm.firesuppress.arminit[1])
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

	-- Intermediet configuration
  local trLArm05 = {unpack(trLArm0)}
  local trRArm05 = {unpack(trRArm0)}
  trLArm05[5] = trLArm1[5]  -- TODO: why pitch???
  trRArm05[5] = trRArm1[5]

  arm_planner:set_hand_mass(0,0)
  arm_planner:set_shoulder_yaw_target(nil, nil) 

  local wrist_seq = {
		{'wrist', trArm05, trRArm05},
    {'wrist', trLArm1, trRArm1},
  }

	-- Initialize planner
  if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "wristyawturn" end  

  hcm.set_fire_model( Config.armfsm.firesuppress.default_model )
  hcm.set_state_proceed(1)  -- proceed after wrist turn
  -- hcm.set_state_proceed(0)  -- Stop after wrist turn

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

  if stage=="wristyawturn" then --Turn yaw angles first    
    if arm_planner:play_arm_sequence(t) then  -- playback DONE      
      if hcm.get_state_proceed()==1 then  -- proceed to next configaration
        print("trRArm:",arm_planner.print_transform(trRArm))
        local arm_seq = {
          {'move',nil,Config.armfsm.firesuppress.arminit[1]},
          {'move',nil,Config.armfsm.firesuppress.arminit[2]},
          {'move',nil,Config.armfsm.firesuppress.arminit[3]},
          {'wrist',nil,Config.armfsm.firesuppress.arminit[4]}
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armup" end
				hcm.set_state_proceed(0)  -- stay after move up
      elseif hcm.get_state_proceed()==-1 then  -- revert to previous configuration
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3]) 
        local wrist_seq = {{"wrist",trLArm0,trRArm0}}
        if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "armbacktoinitpos" end  
      end
    end
  elseif stage=="armup" then
    if arm_planner:play_arm_sequence(t) then   -- Arm already move up
      if hcm.get_state_proceed()==1 then 
        print("trRArm:",arm_planner.print_transform(trRArm))
        arm_planner:set_shoulder_yaw_target(nil,nil)  
        local trRArmTarget = get_fire_tr({0,0,0})  -- input: xyz offset
        local arm_seq = { {'move',nil, trRArmTarget} }     
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "target" end --plan for targetting        
        hcm.set_state_proceed(0) -- Stop after target
      elseif hcm.get_state_proceed()==-1 then 
         
      end
    end        
  
  elseif stage=="target" then  -- aim at the fire   
    if arm_planner:play_arm_sequence(t) then  -- alreay target  
      if hcm.get_state_proceed()==1 then                   
				-- DO NOTHING
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_hand_mass(0,0)
        local arm_seq = {
          {'move',nil,Config.armfsm.firesuppress.arminit[3]},
          {'move',nil,Config.armfsm.firesuppress.arminit[2]},
          {'move',nil,Config.armfsm.firesuppress.arminit[1]},
          {'move',nil,trRArm1}
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then 
					stage = "wristyawturn" 
				  hcm.set_state_proceed(0)  -- Avoid "done"
				end                  
     
      elseif check_override() then --Model modification
        print("target on fire")
				-- Update fire model
        update_override()        
        local trRArmTarget2 = get_fire_tr({0,0,0})
        local arm_seq = {{'move',nil,trRArmTarget2}}
        
        if arm_planner:plan_arm_sequence2(arm_seq) then 
          stage = "target" 
          clear_override()
        else revert_override() end
      end
    end
    
	
	----------------------------------------------------------
	--Backward motions motions
	----------------------------------------------------------
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
