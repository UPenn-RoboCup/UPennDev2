local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local movearm = require'movearm'

local handle_pos, handle_yaw, handle_pitch, handle_radius, turnAngle=0,0,0,0,0
local lShoulderYaw, rShoulderYaw = 0,0
local stage = 1;
local handle_pos_temp

local qJointVelInit = 
  {30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD,
   30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD,}

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  
  mcm.set_arm_handoffset(Config.arm.handoffset.gripper)

  --open gripper
  Body.set_lgrip_percent(0)
  Body.set_rgrip_percent(0)

  --Initial hand angle
  qLArm = Body.get_larm_command_position()
  qRArm = Body.get_rarm_command_position()

  local lhand_rpy0 = {0,0*DEG_TO_RAD, -45*DEG_TO_RAD}
  local rhand_rpy0 = {0,0*DEG_TO_RAD, 45*DEG_TO_RAD}

  qLArm0 = Body.get_inverse_arm_given_wrist( qLArm, {0,0,0, unpack(lhand_rpy0)})
  qRArm0 = Body.get_inverse_arm_given_wrist( qRArm, {0,0,0, unpack(rhand_rpy0)})

  --This works for valvetest
  --New world model for new IK
  hcm.set_wheel_model(
    {0.36,0.00,0.02,   0, 0*DEG_TO_RAD,0.20})

  -- Let's store wheel data here
  local wheel   = hcm.get_wheel_model()
  handle_pos    = vector.slice(wheel,1,3)
  handle_yaw    = wheel[4]
  handle_pitch  = wheel[5]
  handle_radius = wheel[6]
  print("Handle pos (m): ",unpack(handle_pos))
  print("Handle tilt (deg):",handle_pitch*Body.RAD_TO_DEG)
  print("Handle yaw (deg):",handle_yaw*Body.RAD_TO_DEG)
  print("Handle radius (m):",handle_radius)

  

  -- Inner and outer radius
  handle_radius0 = handle_radius 
  handle_radius1 = handle_radius + 0.04

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()    
  lShoulderYaw = qLArm[3]
  rShoulderYaw = qRArm[3]  

  handle_pos0 = {0.24,0,-0.10} --waist position
  handle_pos1 = {handle_pos[1],0,-0.10} --front
  
  turnAngle = 0
  hcm.set_wheel_turnangle(0)


  stage = 1;
end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()    

  if stage==1 then --Change wrist        
    ret = movearm.setArmJoints(qLArm0, qRArm0 ,dt, qJointVelInit)
    if ret==1 then stage=stage+1 end    
  elseif stage==2 then    
    ret = movearm.setArmToWheelPosition(
      handle_pos0, handle_yaw, handle_pitch,
      handle_radius1, turnAngle,dt)    
    if ret==1 then stage=stage+1;     
    end
elseif stage==3 then    
    ret = movearm.setArmToWheelPosition(
      handle_pos1, handle_yaw, handle_pitch,
      handle_radius1, turnAngle,dt)    
    if ret==1 then stage=stage+1;     
    end    
  elseif stage==4 then
    ret = movearm.setArmToWheelPosition(
      handle_pos, handle_yaw, handle_pitch,
      handle_radius1, turnAngle,dt)    
    if ret==1 then stage=stage+1;     
    end  
  elseif stage==5 then
    ret = movearm.setArmToWheelPosition(
      handle_pos, handle_yaw, handle_pitch,
      handle_radius0, turnAngle,dt)
    if ret==-1 then return'reset'
    elseif ret==1 then

      return'done'
    end  
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
