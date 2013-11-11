local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'
local libArmPlan = require 'libArmPlan'
local arm_planner = libArmPlan.new_planner()

--Door opening state using HOOK
local dqArmMax = Config.arm.slow_limit
local dpArmMax = Config.arm.linear_slow_limit
local dDoorAngleMax = 1*math.pi/180

local stage = 1;
local hinge_pos={}
local door_r,grip_offset_x,door_yaw,door_yaw1=0,0,0,0
local door_hand = 0

local handle_clearance = vector.new({0,0,-0.05})
local handle_pullup = vector.new({0,0,0.08})



local plan_failed = false


local trLArm1, trRArm1
local lhand_rpy0, rhand_rpy0

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  --close gripper
  Body.set_lgrip_percent(0.8)
  Body.set_rgrip_percent(0.8)

  --Read door model from shm
  
  door_yaw = hcm.get_door_yaw()    
  
  door_yaw = 0
  
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  lhand_rpy0 = {90*Body.DEG_TO_RAD,0,0}
  rhand_rpy0 = {-90*Body.DEG_TO_RAD,0,0}

  --First init jangles (wrist roll straight)
  qRArm0 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})
  qRArm0[6] = 0,0

  --Second init jangles (wrist roll bent)  
  qLArm1 = qLArm
  qRArm1 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})

  trLArm1 = Body.get_forward_larm(qLArm1)
  trRArm1 = Body.get_forward_rarm(qRArm1)  

--The door we have: hinge height 93.98
--door r: 86.36
  
  local hinge_pos = vector.new({0.55,-1.15,-0.10})

  local hinge_pos = vector.new({0.55,-1.15,-0.12})


--  local hinge_pos = vector.new({0.55,-1.21,0})
  local door_r = 0.86
  local grip_offset_x = -0.05
  local knob_offset_y = 0.05

  local knob_offset_y = 0.15
  
  hcm.set_door_model({
    hinge_pos[1],hinge_pos[2],hinge_pos[3],
    door_r,
    grip_offset_x,
    knob_offset_y})
  hcm.set_door_yaw(0)

  arm_planner:reset_torso_comp(qLArm1, qRArm1)
  arm_planner:save_boundary_condition({qLArm1, qRArm1, qLArm1, qRArm1, {0,0}})
  stage = "wristyawturn";  
end


function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  if plan_failed then return "planfail" end
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  if stage=="wristyawturn" then --Turn yaw angles first
    if movearm.setArmJoints(qLArm,qRArm0,dt, Config.arm.joint_init_limit) ==1 then 
      stage = "wristrollturn"
    end
  elseif stage=="wristrollturn" then   
    if movearm.setArmJoints(qLArm,qRArm1,dt, Config.arm.joint_init_limit) ==1 then
      local trRArmTarget1 = movearm.getDoorHandlePosition(handle_clearance, 0, door_yaw, rhand_rpy0)
      local trRArmTarget2 = movearm.getDoorHandlePosition({0,0,0}, 0, door_yaw, rhand_rpy0)
      local arm_seq = {    
        mass={0,0},
        armseq={
          {trLArm1, trRArmTarget1},
          {trLArm1, trRArmTarget2},      
        }
      }
      if arm_planner:plan_arm_sequence(arm_seq) then stage = "knobhook"  end
    end    
  elseif stage=="knobhook" then --Move the arm forward using IK now     
    if arm_planner:play_arm_sequence(t) then 
      local dooropen_seq = {
        {{{0,0,0},0*Body.DEG_TO_RAD,0}, {{0,0,0},-30*Body.DEG_TO_RAD,0} },
--        {{{0,0,0},-30*Body.DEG_TO_RAD,0}, {{0,0,0},-30*Body.DEG_TO_RAD,15*Body.DEG_TO_RAD} }        
        {{{0,0,0},-30*Body.DEG_TO_RAD,0}, {{0,0,0},-30*Body.DEG_TO_RAD,20*Body.DEG_TO_RAD} }        

      }
      if arm_planner:plan_open_door_sequence(dooropen_seq) then stage = "dooropen"  end
    end
  elseif stage=="dooropen" then --Move the arm forward using IK now     
    if arm_planner:play_arm_sequence(t) then stage = "opendone" end
  end
 
end

function state.exit()  
  print(state._NAME..' Exit' )
end

return state
