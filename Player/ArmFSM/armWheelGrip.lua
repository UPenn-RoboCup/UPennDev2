local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local wheelrotate = require'wheelrotate'

-- Angular velocity limit

local t_init = 5.0
local t_grip = 5.0
local handle_pos, handle_yaw, handle_pitch, handle_radius=0,0,0,0;
local turnAngle = 0
local lShoulderYaw = 0;
local rShoulderYaw = 0;
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
--SJ: Just in case
  if handle_pos[1]==0 then
    handle_pos={0.40,0,0.10}
    handle_yaw=0
    handle_pitch=0
    handle_radius=0.10
    hcm.set_wheel_model({handle_pos[1],handle_pos[2],handle_pos[3],
                        handle_yaw,handle_pitch,handle_radius})
  end

  --open gripper
  Body.set_lgrip_percent(0)
  Body.set_rgrip_percent(0)


-- Inner and outer radius
  handle_radius0 = handle_radius 
  handle_radius1 = handle_radius + 0.08

  stage = 1;

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  lShoulderYaw = qLArm[3];
  rShoulderYaw = qRArm[3];
   
  --hack for now
  hcm.set_joints_shoulderangle(lShoulderYaw)

end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  if stage==1 then
    ret = wheelrotate.setArmToWheelPosition(
      handle_pos, handle_yaw, handle_pitch,
      handle_radius1, turnAngle,dt,
      lShoulderYaw, rShoulderYaw)
    if ret==1 then stage=stage+1; 
    elseif ret==-1 then 
      return'reset'
    end
  elseif stage==2 then
    ret = wheelrotate.setArmToWheelPosition(
      handle_pos, handle_yaw, handle_pitch,
      handle_radius0, turnAngle,dt,
      lShoulderYaw, rShoulderYaw)
    if ret==-1 then     
      return'reset'
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