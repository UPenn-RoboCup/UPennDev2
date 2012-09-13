----------------------------------------------------------------------
-- Oscillatory Walking
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

-- define default parameters
walk.parameters = {
  x_offset              = 0.025, -- meters
  y_offset              = 0.1,   -- meters
  z_offset              = -0.75, -- meters
  x_swing_ratio         = 0,     -- ratio
  y_swing_amplitude     = 0,     -- meters
  z_swing_amplitude     = 0,     -- meters
  step_amplitude        = 0,     -- meters
  hip_pitch_offset      = 0,     -- radians
  period_time           = 1,     -- seconds
  dsp_ratio             = 0.25,  -- ratio
}

-- load config parameters
for k,v in pairs(walk.parameters) do
   walk.parameters[k] = Config.walk.parameters[k] or v
end

-- define local copies
local x_offset          = walk.parameters.x_offset
local y_offset          = walk.parameters.y_offset
local z_offset          = walk.parameters.z_offset
local x_swing_ratio     = walk.parameters.x_swing_ratio
local y_swing_amplitude = walk.parameters.y_swing_amplitude
local z_swing_amplitude = walk.parameters.z_swing_amplitude
local step_amplitude    = walk.parameters.step_amplitude
local hip_pitch_offset  = walk.parameters.hip_pitch_offset
local period_time       = walk.parameters.period_time
local dsp_ratio         = walk.parameters.dsp_ratio

-- initialize control variables
local is_walking        = false
local stop_request      = false
local velocity          = vector.new{0, 0, 0}
local t0                = Body.get_time()
local q0                = scm:get_joint_position('legs')
local qStance           = vector.copy(q0)

-- Private
----------------------------------------------------------------------

local function update_parameters()
  -- update local gait parameters to current settings

  x_offset = walk.parameters.x_offset
  y_offset = walk.parameters.y_offset
  z_offset = walk.parameters.z_offset
  x_swing_ratio = walk.parameters.x_swing_ratio
  y_swing_amplitude = walk.parameters.y_swing_amplitude
  z_swing_amplitude = walk.parameters.z_swing_amplitude
  step_amplitude = walk.parameters.step_amplitude
  hip_pitch_offset = walk.parameters.hip_pitch_offset
  period_time = walk.parameters.period_time
  dsp_ratio = walk.parameters.dsp_ratio
end

local function zero_sin(theta, alpha)
  -- generate a sinusoidal waveform with extended zero regions 
  -- alpha in [0, 1] determines the percentage of deadband in a cycle
  -- where zero_sin(theta, 0) = sin(theta)

  alpha = util.min{util.max{alpha, 0}, 1}
  local PI = math.pi
  local theta = theta % (2*PI)
  local offset = PI/2*alpha
  if ((theta > offset)
  and (theta < PI - offset)) then
    return math.sin((theta - offset)/(1 - alpha))
  elseif ((theta > PI + offset)
  and (theta < 2*PI - offset)) then
    return math.sin((theta - (PI + offset))/(alpha - 1))
  end
  return 0
end

local function unity_sin(theta, alpha)
  -- generate a sinusoidal waveform with extended max and min regions
  -- alpha in [0, 1] determines the percentage of deadband in a cycle
  -- where unity_sin(theta, 0) = sin(theta)

  alpha = util.min{util.max{alpha, 0}, 1}
  local PI = math.pi
  local theta = theta % (2*PI)
  local offset = PI/2*alpha
  if ((theta < PI/2 - offset) 
  or  (theta > 3*PI/2 + offset)) then
    return math.sin((theta - 2*PI)/(1 - alpha))
  elseif ((theta > PI/2 + offset)
  and (theta < 3*PI/2 - offset)) then
    return -math.sin((theta - PI)/(1 - alpha))
  end
  return 1
end

local function zero_cos(theta, alpha)
  return zero_sin(theta + math.pi/2, alpha) 
end

local function unity_cos(theta, alpha)
  return unity_sin(theta + math.pi/2, alpha) 
end

-- Public
----------------------------------------------------------------------

function walk:start()
  self.active = true
  update_parameters()
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
  self.active = false
  -- configure write access
  self:set_joint_access(0, 'all')
  self:set_joint_access(1, 'legs')
end

function walk:update()
  if self.active then
    if is_walking then
      -- get sensor readings 
      local gyro = sensor:get_ahrs('gyro')
      local tilt = sensor:get_ahrs('euler')

      -- update endpoint trajectories
      local ph = 2*math.pi/period_time*(Body.get_time() - t0)
      local x_swing_amplitude = velocity[1]*x_swing_ratio
      local x_swing = x_swing_amplitude*math.cos(2*ph)
      local y_swing = y_swing_amplitude*math.sin(ph)
      local z_swing = z_swing_amplitude*math.cos(2*ph)
      local l_foot_pos = vector.zeros(6)
      l_foot_pos[1] = x_offset + x_swing - velocity[1]*unity_cos(ph, dsp_ratio)
      l_foot_pos[2] = y_offset + y_swing - velocity[2]*unity_cos(ph, dsp_ratio)
      l_foot_pos[3] = z_offset - z_swing + step_amplitude*zero_sin(ph, dsp_ratio)
      l_foot_pos[6] = -velocity[3]*unity_cos(ph, dsp_ratio)
      local r_foot_pos = vector.zeros(6) 
      r_foot_pos[1] = x_offset + x_swing + velocity[1]*unity_cos(ph, dsp_ratio)
      r_foot_pos[2] =-y_offset + y_swing + velocity[2]*unity_cos(ph, dsp_ratio)
      r_foot_pos[3] = z_offset - z_swing - step_amplitude*zero_sin(ph, dsp_ratio)
      r_foot_pos[6] = velocity[3]*unity_cos(ph, dsp_ratio)
      local torso_pos = vector.zeros(6) 
      torso_pos[5] = hip_pitch_offset

      -- calculate inverse kinematics
      local qLegs = Kinematics.inverse_legs(l_foot_pos, r_foot_pos, torso_pos)

      -- write joint angles to actuator shared memory 
      actuator:set_joint_position(qLegs, 'legs')

      -- check stop conditions (only stop during double stance)
      if (math.abs(math.sin(ph)) < 0.025 and stop_request) then
         self.active = false
         is_walking = false
         stop_request = false
      end

      -- update gait parameters at the end of each cycle 
      if (ph > 2*math.pi) then
        update_parameters()
        t0 = Body.get_time() 
      end
    else
      local t = Body.get_time()
      if (t - t0 > 2) then
        -- begin walking
        is_walking = true
        t0 = t
      elseif stop_request then
        -- stop
        self.active = false
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
  self.active = false
end

return walk
