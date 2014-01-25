local state = {}
state._NAME = ...
require'hcm'
require'mcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()
local T      = require'Transform'

local qLArm0,qRArm0, trLArm0, trRArm0, trLArm1, trARArm1

--Initial hand angle
local lhand_rpy = Config.armfsm.toolgrip.lhand_rpy
local rhand_rpy = Config.armfsm.toolgrip.rhand_rpy


local gripL, gripR = 1,1
local stage
local debugdata

local wristPitch = 0
local wristYaw = 0


local function get_tool_tr(tooloffset)

  local handrpy = rhand_rpy
  local tool_model = hcm.get_tool_model()
  local hand_pos = vector.slice(tool_model,1,3) + 
    vector.new({tooloffset[1],tooloffset[2],tooloffset[3]})  
  local tool_tr = {hand_pos[1],hand_pos[2],hand_pos[3],
                    handrpy[1],handrpy[2],handrpy[3] + tool_model[4]}

  print("hand transform:",arm_planner.print_transform(tool_tr))                    
  return tool_tr
end

local function get_tool_tr_2()
  local handrpy = rhand_rpy
  local tool_model = hcm.get_tool_model()

  local trHand = T.eye()
      *T.trans(tool_model[1],tool_model[2],tool_model[3])
      *T.rotY(wristPitch)
      *T.rotZ(wristYaw+45*math.pi/180)

  return T.position6D(trHand)

--[[
  local hand_pos = vector.slice(tool_model,1,3) + 
    vector.new({tooloffset[1],tooloffset[2],tooloffset[3]})  
  local tool_tr = {hand_pos[1],hand_pos[2],hand_pos[3],
                    handrpy[1],handrpy[2],handrpy[3] + tool_model[4]}

  print("hand transform:",arm_planner.print_transform(tool_tr))                    
  return tool_tr

--]]
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

local function update_override(disable_rotate)
  local override = hcm.get_state_override()
  local tool_model = hcm.get_tool_model()


--[[
  tool_model[1],tool_model[2],tool_model[3]=
 
  tool_model[1] + override[1],
  tool_model[2] + override[2],
  tool_model[3] + override[3]
--]]


  local trArmCurrent = hcm.get_hands_right_tr()        

  tool_model[1],tool_model[2],tool_model[3]=
 
  trArmCurrent[1] + override[1],
  trArmCurrent[2] + override[2],
  trArmCurrent[3] + override[3]

  if not disable_rotate then
--print("upd: old model",unpack(hcm.get_tool_model() ))

  wristPitch = wristPitch + override[5]*2.5*Body.DEG_TO_RAD 
  wristYaw = wristYaw + override[6]*2.5*Body.DEG_TO_RAD 

  wristPitch= math.max(-30*Body.DEG_TO_RAD,math.min(30*Body.DEG_TO_RAD,wristPitch))
  wristYaw= math.max(-70*Body.DEG_TO_RAD,math.min(20*Body.DEG_TO_RAD,wristYaw))

  end



--SJ: this is weird, we need to read once to update the shm
  hcm.get_tool_model()
  hcm.set_tool_model(tool_model)

  print( util.color('Tool model:','yellow'), 
      string.format("%.2f %.2f %.2f / %.1f",
        tool_model[1],tool_model[2],tool_model[3],
        tool_model[4]*180/math.pi ))
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
  tool_model[4] - override[6]*5*Body.DEG_TO_RAD, --yaw

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

local function confirm_override()
  hcm.set_state_override({0,0,0,0,0,0,0})
end

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  mcm.set_arm_lhandoffset(Config.arm.handoffset.gripper)
  mcm.set_arm_rhandoffset(Config.arm.handoffset.gripper)

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  qLArm0 = qLArm
  qRArm0 = qRArm
  trLArm0 = Body.get_forward_larm(qLArm0)
  trRArm0 = Body.get_forward_rarm(qRArm0)  

  --Initial arm joint angles after rotating wrist
  qLArm1 = Body.get_inverse_arm_given_wrist( qLArm, Config.armfsm.toolgrip.larm[1])
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, Config.armfsm.toolgrip.arminit[1])
  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

  local trLArm05 = {unpack(trLArm0)}
  local trRArm05 = {unpack(trRArm0)}
  trLArm05[5] = trLArm1[5]
  trRArm05[5] = trRArm1[5]

  arm_planner:set_hand_mass(0,0)
--  arm_planner:set_shoulder_yaw_target(qLArm0[3], nil) 
  arm_planner:set_shoulder_yaw_target(nil, nil) 

  local wrist_seq = {
    {'wrist',trLArm1, trRArm1},
  }

  --local wrist_seq = {{'wrist',trLArm1,trRArm1}}
  if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "wristyawturn" end  

  --hcm.set_tool_model(Config.armfsm.toolgrip.default_model)
  hcm.set_tool_model(Config.armfsm.firesuppress.default_model)  

--print("tool model:",unpack(Config.armfsm.firesuppress.default_model))

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

  if stage=="wristyawturn" then --Turn yaw angles first    
    if arm_planner:play_arm_sequence(t) then       
      if hcm.get_state_proceed()==1 then 
--        print("trLArm:",arm_planner.print_transform(trLArm))
        print("trRArm:",arm_planner.print_transform(trRArm))
--        arm_planner:set_shoulder_yaw_target(qLArm0[3],nil)        
        local arm_seq = {
          {'move',nil,Config.armfsm.firesuppress.arminit[1]},
          {'move',nil,Config.armfsm.firesuppress.arminit[2]},
          {'move',nil,Config.armfsm.firesuppress.arminit[3]},
          {'wrist',nil,Config.armfsm.firesuppress.arminit[4]},
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "armup" end
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_shoulder_yaw_target(qLArm0[3],qRArm0[3]) 
        --local wrist_seq = {{"wrist",nil,trRArm0}}
        local wrist_seq = {{"wrist",trLArm0,trRArm0}}
        if arm_planner:plan_arm_sequence2(wrist_seq) then stage = "armbacktoinitpos" end  
      end
    end
  elseif stage=="armup" then
    if arm_planner:play_arm_sequence(t) then 
      if hcm.get_state_proceed()==1 then 
        print("trRArm:",arm_planner.print_transform(trRArm))
        --arm_planner:set_shoulder_yaw_target(qLArm0[3],nil)
        arm_planner:set_shoulder_yaw_target(nil,nil)        
        local trRArmTarget = get_tool_tr({0,0,0})
        local arm_seq = { {'move',nil, trRArmTarget} }     
--print("armTarget:",unpack(trRArmTarget))        
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "grab" end          
        hcm.set_state_proceed(0) --stop here
      elseif hcm.get_state_proceed()==-1 then 
         
      end
    end        
  
  elseif stage=="grab" then --Grip the object   
    if arm_planner:play_arm_sequence(t) then    
      if hcm.get_state_proceed()==1 then        
--[[        
        arm_planner:set_hand_mass(0,1)
        local trRArmTarget3 = get_tool_tr({0,0,0})
        local arm_seq = {{'move',nil, trRArmTarget3}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "lift" end
        hcm.set_state_proceed(0) --stop here
--]]        
print("trLArm:",arm_planner.print_transform(trLArm))
print("trRArm:",arm_planner.print_transform(trRArm))
        local arm_seq = {
--          {'move',nil,Config.armfsm.firesuppress.arminit[5]},
--          {'move',Config.armfsm.firesuppress.larmtrigger[1],nil},

          {'move',Config.armfsm.firesuppress.larmtrigger[1],nil},
          {'move',Config.armfsm.firesuppress.larmtrigger[2],Config.armfsm.firesuppress.arminit[5]},
          {'wrist',Config.armfsm.firesuppress.larmtrigger[3],nil},
          {'move',Config.armfsm.firesuppress.larmtrigger[3],nil},
          {'move',Config.armfsm.firesuppress.larmtrigger[4],nil},--move up a bit

          {'wrist',Config.armfsm.firesuppress.larmtrigger[2],nil},
          {'move',Config.armfsm.firesuppress.larmtrigger[1],Config.armfsm.firesuppress.arminit[6]},
          {'move',Config.armfsm.firesuppress.larmtrigger[5],nil},

        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "lift" end
        hcm.set_state_proceed(0) --stop here

      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_hand_mass(0,0)
        local arm_seq = {
          {'wrist',nil,Config.armfsm.firesuppress.arminit[3]},
          {'move',nil,Config.armfsm.firesuppress.arminit[3]},
          {'move',nil,Config.armfsm.firesuppress.arminit[2]},
          {'move',nil,Config.armfsm.firesuppress.arminit[1]},
          {'move',nil,trRArm1}
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "wristyawturn" end                  
     
      elseif check_override() then --Model modification
        print("grab")
--        local trRArmTarget2 = get_tool_tr({0,0,0})       

        update_override(1)        


        


--        local trRArmTarget2 = get_tool_tr({0,0,0})
        local trRArmTarget2 = get_tool_tr_2()

        local arm_seq = {{'move',nil,trRArmTarget2}}
--[[
        if check_override_rotate() then
          arm_seq = {{'wrist',nil,trRArmTarget2}}
        end
--]]        
        if arm_planner:plan_arm_sequence2(arm_seq) then 
          stage = "grab" 
          confirm_override()
        else revert_override() end
      end
    end











    
  elseif stage=="lift" then
    if arm_planner:play_arm_sequence(t) then    
--      print("trLArm:",arm_planner.print_transform(trLArm))
--      print("trRArm:",arm_planner.print_transform(trRArm))
 
  
--      if hcm.get_state_proceed()==1 then        
if false then
        arm_planner:set_hand_mass(0,2)
        print("trRArm:",arm_planner.print_transform(trRArm))
        

        --Going back to the init position
        local trRArmTarget1 = Config.armfsm.toolgrip.armpull[1]
        local trRArmTarget2 = Config.armfsm.toolgrip.armpull[2]
        local trRArmTarget3 = Config.armfsm.toolgrip.armpull[3]
        local trRArmTarget4 = Config.armfsm.toolgrip.armhold

        local arm_seq = {
          {'move',nil,trRArmTarget1},
          {'wrist',nil,trRArmTarget2},
          {'move',nil,trRArmTarget2},
          {'move',nil,trRArmTarget3},
          {'wrist',nil,trRArmTarget4},
          {'move',nil,trRArmTarget4}          
        }
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "liftpull" end
      elseif hcm.get_state_proceed()==-1 then 
        arm_planner:set_hand_mass(0,1)   
        local trRArmTarget3 = get_tool_tr({0,0,0})
        local arm_seq = {{'move',nil,trRArmTarget3}}
        if arm_planner:plan_arm_sequence2(arm_seq) then stage = "grab" end
      
      elseif check_override() then --Model modification

--       print("grab")
--        local trRArmTarget2 = get_tool_tr({0,0,0})
        update_override()        
        local trRArmTarget2 = get_tool_tr_2()
        local arm_seq = {{'move',nil,trRArmTarget2}}
        if check_override_rotate() then
          arm_seq = {{'wrist',nil,trRArmTarget2}}
        end
        if arm_planner:plan_arm_sequence2(arm_seq) then 
          stage = "lift" 
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

----------------------------------------------------------
--Backward motions motions
----------------------------------------------------------
  
  elseif stage=="armbacktoinitpos" then 
    if arm_planner:play_arm_sequence(t) then return "done" end
  end


  hcm.set_motion_wristYaw(wristYaw+45*Body.DEG_TO_RAD)
  hcm.set_motion_wristPitch(wristPitch)

  
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
