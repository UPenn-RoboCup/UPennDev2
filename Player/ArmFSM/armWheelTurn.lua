local state = {}
state._NAME = ...
require'hcm'
local vector = require'vector'
local util   = require'util'
local movearm = require'movearm'

local handle_pos, handle_yaw, handle_pitch, handle_radius, turnAngle=0,0,0,0,0
local lShoulderYaw, rShoulderYaw = 0,0

local dqArmMax = Config.arm.fast_limit 
local dpArmMax = Config.arm.linear_slow_limit

local dturnAngleMax = 3*math.pi/180 -- 3 deg per sec
local dpHandleMax = {0.01, 0.01, 0.01}
local drHandleMax = 0.01
local daHandleMax = 3*math.pi/180 

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

  handle_pos = util.approachTol(handle_pos, handle_pos1, dpHandleMax,dt)
  handle_yaw = util.approachTol(handle_yaw,handle_yaw1, daHandleMax,dt)
  handle_pitch = util.approachTol(handle_pitch,handle_pitch1, daHandleMax,dt)
  handle_radius = util.approachTol(handle_radius, handle_radius1,drHandleMax,dt)    
  turnAngle,doneA = util.approachTol(turnAngle,turnAngle1,dturnAngleMax, dt )
  
  lShoulderYaw = hcm.get_joints_qlshoulderyaw()
  rShoulderYaw = hcm.get_joints_qrshoulderyaw()


  ret = movearm.setArmToWheelPosition(
    handle_pos, handle_yaw, handle_pitch,
    handle_radius, turnAngle,dt,
    lShoulderYaw, rShoulderYaw)
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state