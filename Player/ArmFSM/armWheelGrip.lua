local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local movearm = require'movearm'

local handle_pos, handle_yaw, handle_pitch, handle_radius, turnAngle=0,0,0,0,0
local lShoulderYaw, rShoulderYaw = 0,0
local stage = 1;
local handle_pos_temp

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

--temporary hack for the ankle 

--[[
local x0,z0 = 0.41, 1.02-0.928 + 0.04
local rpy = Body.get_sensor_rpy()
local bodyTilt = -rpy[2]
bodyTilt = bodyTilt - 3*math.pi/180; --to account for bodytilt ing front

local wheel_model = vector.new({
        x0*math.cos(bodyTilt) + z0*math.sin(bodyTilt),
        0,
        -x0*math.sin(bodyTilt)+z0*math.cos(bodyTilt),
        0,  
        bodyTilt,
        0.20})
hcm.set_wheel_model(wheel_model)
--]] 
  
  -- Let's store wheel data here
  local wheel   = hcm.get_wheel_model()
  handle_pos    = vector.slice(wheel,1,3)
  handle_yaw    = wheel[4]
  handle_pitch  = wheel[5]
  handle_radius = wheel[6]
  print("Handle pos: ",unpack(handle_pos))
  print("Handle tilt:",handle_pitch*Body.RAD_TO_DEG)
  print("Handle yaw:",handle_yaw*Body.RAD_TO_DEG)
  print("Handle radius:",handle_radius)

  --open gripper
  Body.set_lgrip_percent(0)
  Body.set_rgrip_percent(0)

  -- Inner and outer radius
  handle_radius0 = handle_radius 
  handle_radius1 = handle_radius + 0.04

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()    
  lShoulderYaw = qLArm[3]
  rShoulderYaw = qRArm[3]  

  handle_pos0 = {0.24,0,-0.10} --waist position
  handle_pos1 = {handle_pos[1],0,-0.10} --front
  

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
--[[    
    dqWristMax=vector.new({0,0,0,0,
       15*Body.DEG_TO_RAD,15*Body.DEG_TO_RAD,15*Body.DEG_TO_RAD})
    qL = Body.get_inverse_arm_given_wrist( qLArm,       {0,0,0,0,-Config.walk.bodyTilt,-45*Body.DEG_TO_RAD})
    qR = Body.get_inverse_arm_given_wrist( qRArm,       {0,0,0,0,-Config.walk.bodyTilt,45*Body.DEG_TO_RAD})
    ret= movearm.setArmJoints(qL, qR, dt, dqWristMax)
    
    if ret==1 then stage=stage+1;     
    end
--]]

    stage=stage+1    
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
