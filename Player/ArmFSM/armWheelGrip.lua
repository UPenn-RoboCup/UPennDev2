local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local movearm = require'movearm'

local handle_pos, handle_yaw, handle_pitch, handle_radius, turnAngle=0,0,0,0,0
local lShoulderYaw, rShoulderYaw = 0,0
local stage = 1;

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  
  -- Let's store wheel data here
  local wheel   = hcm.get_wheel_model()
  handle_pos    = vector.slice(wheel,1,3)
  handle_yaw    = wheel[4]
  handle_pitch  = wheel[5]
  handle_radius = wheel[6]
  print("Handle model:",wheel)

  --open gripper
  Body.set_lgrip_percent(0)
  Body.set_rgrip_percent(0)

  -- Inner and outer radius
  handle_radius0 = handle_radius 
  handle_radius1 = handle_radius + 0.08

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()    
  lShoulderYaw = qLArm[3]
  rShoulderYaw = qRArm[3]  

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
    dqWristMax=vector.new({0,0,0,0,
       10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD,10*Body.DEG_TO_RAD})
    qL = Body.get_inverse_arm_given_wrist( qLArm,       {0,0,0,0,0,-45*Body.DEG_TO_RAD})
    qR = Body.get_inverse_arm_given_wrist( qRArm,       {0,0,0,0,0,45*Body.DEG_TO_RAD})
    if movearm.setArmJoints(qL, qR, dt, dqWristMax)==1 then
      stage=stage+1;       
    end        
  elseif stage==2 then
    --[[
    ret = movearm.setArmToWheelPosition(
      handle_pos, handle_yaw, handle_pitch,
      handle_radius1, turnAngle,dt,
      lShoulderYaw, rShoulderYaw)
--]]
    handle_pos_temp={handle_pos[1],handle_pos[2],-0.10}

    ret = movearm.setArmToWheelPosition(
      handle_pos_temp, handle_yaw, handle_pitch,
      handle_radius1, turnAngle,dt)    
    if ret==1 then stage=stage+1;     
    end
  elseif stage==3 then
    ret = movearm.setArmToWheelPosition(
      handle_pos, handle_yaw, handle_pitch,
      handle_radius1, turnAngle,dt)    
    if ret==1 then stage=stage+1;     
    end
  elseif stage==4 then
    ret = movearm.setArmToWheelPosition(
      handle_pos, handle_yaw, handle_pitch,
      handle_radius0, turnAngle,dt)
--    ,      lShoulderYaw, rShoulderYaw)
    if ret==-1 then return'reset'
    elseif ret==1 then
      Body.set_lgrip_percent(1)
      Body.set_rgrip_percent(1)
      return'done'
    end  
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state