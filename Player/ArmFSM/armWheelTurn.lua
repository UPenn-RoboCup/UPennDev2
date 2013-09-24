local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local wheelrotate = require'wheelrotate'

-- Angular velocity limit

local handle_pos, handle_yaw, handle_pitch, handle_radius=0,0,0,0;
local turnAngle = 0
local lShoulderYaw = 0;
local rShoulderYaw = 0;
local stage = 1;

local qLArmTarget = Config.arm.qLArmInit[3]
local qRArmTarget = Config.arm.qRArmInit[3]
local dqArmMax = Config.arm.fast_limit
local dpArmMax = Config.arm.linear_slow_limit
local dturnAngleMax = 3*math.pi/180 -- 3 deg per sec

function state.entry()
  print(state._NAME..' Entry' )
  -- Update the time of entry
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  local wheel   = hcm.get_wheel_model()
  turnAngle = hcm.get_wheel_turnangle()
  handle_pos    = vector.slice(wheel,1,3)
  handle_yaw    = wheel[4]
  handle_pitch  = wheel[5]
  handle_radius = wheel[6]

end

function state.update()
--  print(state._NAME..' Update' )
  -- Get the time of update
  local t  = Body.get_time()
  local dt = t - t_update
  -- Save this at the last update time
  t_update = t
  --if t-t_entry > timeout then return'timeout' end

  --Update handle model using interpolation
 
  local wheel   = hcm.get_wheel_model()
  turnAngle1 = hcm.get_wheel_turnangle()
  handle_pos1    = vector.slice(wheel,1,3)
  handle_yaw1    = wheel[4]
  handle_pitch1  = wheel[5]
  handle_radius1 = wheel[6]

  handle_pos = util.approachTol(handle_pos, handle_pos1, 
    {0.01,0.01,0.01},dt)
  handle_yaw = util.approachTol(handle_yaw,handle_yaw1,
    3*Body.DEG_TO_RAD,dt)
  handle_pitch = util.approachTol(handle_pitch,handle_pitch1,
    3*Body.DEG_TO_RAD,dt)
  handle_radius = util.approachTol(handle_radius, handle_radius1,0.01,dt)  
  
  turnAngle,doneA = util.approachTol(turnAngle,turnAngle1,dturnAngleMax, dt )
  
  --TODO: use interpolation here too  
  lShoulderYaw = hcm.get_joints_shoulderangle()
  rShoulderYaw = - lShoulderYaw;

  ret = wheelrotate.setArmToWheelPosition(
    handle_pos, handle_yaw, handle_pitch,
    handle_radius, turnAngle,dt,
    lShoulderYaw, rShoulderYaw)
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state