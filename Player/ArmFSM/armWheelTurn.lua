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
  handle_pos    = vector.slice(wheel,1,3)
  handle_yaw    = wheel[4]
  handle_pitch  = wheel[5]
  handle_radius = wheel[6]
  turnAngle = 0
  hcm.set_wheel_turnangle(0)
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
  local turnAngle1 = hcm.get_wheel_turnangle()
  local handle_pos1    = vector.slice(wheel,1,3)
  local handle_yaw1    = wheel[4]
  local handle_pitch1  = wheel[5]
  local handle_radius1 = wheel[6]

  local handle_pos2 = util.approachTol(handle_pos, handle_pos1, dpHandleMax,dt)
  local handle_yaw2 = util.approachTol(handle_yaw,handle_yaw1, daHandleMax,dt)
  local handle_pitch2 = util.approachTol(handle_pitch,handle_pitch1, daHandleMax,dt)
  local handle_radius2 = util.approachTol(handle_radius, handle_radius1,drHandleMax,dt)    
  local turnAngle2,doneA = util.approachTol(turnAngle,turnAngle1,dturnAngleMax, dt )
  
  --Adaptive shoulder yaw angle
  ret = movearm.setArmToWheelPosition(
    handle_pos2, handle_yaw2, handle_pitch2,
    handle_radius2, turnAngle2,dt)
  if ret==-1 then    
    hcm.set_wheel_model({handle_pos[1],handle_pos[2],handle_pos[3],
     handle_yaw,handle_pitch,handle_radius})
    hcm.set_wheel_turnangle(turnAngle)    
  else
    handle_pos,handle_yaw,handle_pitch,handle_radius,turnAngle=
      handle_pos2,handle_yaw2,handle_pitch2,handle_radius2,turnAngle2 
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state