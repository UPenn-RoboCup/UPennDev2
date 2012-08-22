----------------------------------------------------------------------
-- Sine Wave Walking
----------------------------------------------------------------------

require('util')
require('Body')
require('vector')
require('Config')
require('Kinematics')
require('MotionState')

-- Setup 
----------------------------------------------------------------------

walk = MotionState.new(...)
local sensor = walk.sensor
local actuator = walk.actuator

-- Gait Parameters
local parameters = Config.walk.parameters
local x_offset = parameters.x_offset or 0.025 -- meters
local y_offset = parameters.y_offset or 0.1
local z_offset = parameters.z_offset or -0.75
local step_amplitude = parameters.step_amplitude or 0
local x_swing_ratio = parameters.x_swing_ratio or 0
local y_swing_amplitude = parameters.y_swing_amplitude or 0
local z_swing_amplitude = parameters.z_swing_amplitude or 0
local hip_pitch_offset = parameters.hip_pitch_offset or 0 -- radians
local period_time = parameters.period_time or 1 -- seconds

local is_walking = false
local stop_request = false
local velocity = vector.new{0, 0, 0}
local t0 = Body.get_time()
local q0 = scm:get_joint_position('legs')
local qStance = vector.copy(q0)

-- Interface
----------------------------------------------------------------------

function walk:start()
  walk.active = true
  t0 = Body.get_time()
  local l_foot_pos = {x_offset, y_offset, z_offset, 0, 0, 0} 
  local r_foot_pos = {x_offset,-y_offset, z_offset, 0, 0, 0} 
  local torso_pos = {0, 0, 0, 0, hip_pitch_offset, 0}
  q0 = scm:get_joint_position('legs')
  qStance = Kinematics.inverse_legs(l_foot_pos, r_foot_pos, torso_pos)
end

function walk:stop()
  stop_request = true
end

function walk:set_velocity(vx, vy, va)
  velocity[1] = vx
  velocity[2] = vy
  velocity[3] = va 
end

function walk:get_velocity()
  return vector.copy(velocity)
end

function walk:entry()
  walk.active = false
  -- configure write access
  self:set_joint_access(0, 'all')
  self:set_joint_access(1, 'legs')
end

function walk:update()
  if walk.active then
    if is_walking then
      -- get sensor readings 
      local gyro = sensor:get_ahrs('gyro')
      local tilt = sensor:get_ahrs('euler')

      -- update endpoint trajectories
      local ph = 2*math.pi/period_time*(Body.get_time()-t0)
      local x_swing_amplitude = velocity[1]*x_swing_ratio
      local x_swing = x_swing_amplitude*math.cos(2*ph) 
      local y_swing = y_swing_amplitude*math.sin(ph)
      local z_swing = z_swing_amplitude*math.cos(2*ph)
      local l_foot_pos = vector.zeros(6) 
      l_foot_pos[1] = x_offset + x_swing - velocity[1]*math.cos(ph)
      l_foot_pos[2] = y_offset + y_swing - velocity[2]*math.cos(ph)
      l_foot_pos[3] = z_offset - z_swing + step_amplitude*math.sin(ph)
      l_foot_pos[6] = -velocity[3]*math.cos(ph)
      local r_foot_pos = vector.zeros(6) 
      r_foot_pos[1] = x_offset + x_swing + velocity[1]*math.cos(ph)
      r_foot_pos[2] =-y_offset + y_swing + velocity[2]*math.cos(ph)
      r_foot_pos[3] = z_offset - z_swing - step_amplitude*math.sin(ph)
      r_foot_pos[6] = velocity[3]*math.cos(ph)
      local torso_pos = vector.zeros(6) 
      torso_pos[5] = hip_pitch_offset

      -- calculate inverse kinematics
      local qLegs = Kinematics.inverse_legs(l_foot_pos, r_foot_pos, torso_pos)

      -- write joint angles to actuator shared memory 
      actuator:set_joint_position(qLegs, 'legs')

      -- stop walking
      if (stop_request and math.abs(math.sin(ph)) < 0.025) then
        walk.active = false
        is_walking = false
        stop_request = false
      end
    else
      local t = Body.get_time()
      if (t - t0 > 2) then
        -- begin walking
        is_walking = true
        t0 = t
      elseif stop_request then
        -- stop walking
        walk.active = false
        stop_request = false
      else
        -- goto initial walking stance 
        local d = util.min{(t - t0)/2, 1}
        local q = q0 + d*(vector.new(qStance) - vector.new(q0))
        actuator:set_joint_position(q, 'legs')
      end
    end
  end
end

function walk:exit()
  walk.active = false
end

return walk
