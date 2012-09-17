----------------------------------------------------------------------
-- Oscillatory Walking
----------------------------------------------------------------------

require('util')
require('Body')
require('vector')
require('Config')
require('waveform')
require('Kinematics')
require('MotionState')

-- Setup 
----------------------------------------------------------------------

walk = MotionState.new(...)
local sensor = walk.sensor
local actuator = walk.actuator
walk:set_joint_access(0, 'all')
walk:set_joint_access(1, 'legs')

-- define default parameters
walk.parameters = {
  x_offset              = 0.025, -- meters
  y_offset              = 0.1,   -- meters
  z_offset              = -0.75, -- meters
  a_offset              = 0,     -- radians
  hip_pitch_offset      = 0,     -- radians
  x_shift_ratio         = 0.5,   -- ratio
  y_shift_ratio         = 0.5,   -- ratio
  y_swing_amplitude     = 0,     -- meters
  z_swing_amplitude     = 0,     -- meters
  r_swing_amplitude     = 0,     -- radians
  step_amplitude        = 0,     -- meters
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
local a_offset          = walk.parameters.a_offset
local hip_pitch_offset  = walk.parameters.hip_pitch_offset
local x_shift_ratio     = walk.parameters.x_shift_ratio
local y_shift_ratio     = walk.parameters.y_shift_ratio
local y_swing_amplitude = walk.parameters.y_swing_amplitude
local z_swing_amplitude = walk.parameters.z_swing_amplitude
local r_swing_amplitude = walk.parameters.r_swing_amplitude
local step_amplitude    = walk.parameters.step_amplitude
local period_time       = walk.parameters.period_time
local dsp_ratio         = walk.parameters.dsp_ratio

-- initialize control variables
local active            = false
local stop_request      = false
local start_request     = false
local velocity          = vector.new{0, 0, 0}
local t0                = Body.get_time()
local q0                = scm:get_joint_position('legs')
local qStance           = vector.copy(q0)

local step_sin = waveform.step_sin
local step_cos = waveform.step_cos
local stride_sin = waveform.stride_sin
local stride_cos = waveform.stride_cos

-- Private
----------------------------------------------------------------------

local function update_gait_parameters()
  -- update local gait parameters

  x_offset = walk.parameters.x_offset
  y_offset = walk.parameters.y_offset
  z_offset = walk.parameters.z_offset
  a_offset = walk.parameters.a_offset
  hip_pitch_offset = walk.parameters.hip_pitch_offset
  x_shift_ratio = walk.parameters.x_shift_ratio
  y_shift_ratio = walk.parameters.y_shift_ratio
  y_swing_amplitude = walk.parameters.y_swing_amplitude
  z_swing_amplitude = walk.parameters.z_swing_amplitude
  r_swing_amplitude = walk.parameters.r_swing_amplitude
  step_amplitude = walk.parameters.step_amplitude
  period_time = walk.parameters.period_time
  dsp_ratio = walk.parameters.dsp_ratio
end

local function update_stance_parameters()
  -- update local stance parameters

  local l_foot_pos = {x_offset, y_offset, z_offset, 0, 0, 0} 
  local r_foot_pos = {x_offset,-y_offset, z_offset, 0, 0, 0} 
  local torso_pos = {0, 0, 0, 0, hip_pitch_offset, 0}
  q0 = scm:get_joint_position('legs')
  qStance = Kinematics.inverse_legs(l_foot_pos, r_foot_pos, torso_pos)
end

-- Public
----------------------------------------------------------------------

function walk:start()
  -- issue start request
  start_request = true
end

function walk:stop()
  -- issue stop request
  stop_request = true
end

function walk:is_active()
  -- return true if gait is active
  return active
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
  self.running = true
  active = false
  update_gait_parameters()
  update_stance_parameters()
  t0 = Body.get_time()
end

function walk:update()
  if active then
    -- get sensor readings 
    local gyro = sensor:get_ahrs('gyro')
    local tilt = sensor:get_ahrs('euler')

    -- update phase
    local ph = 2*math.pi/period_time*(Body.get_time() - t0)

    -- update left foot trajectory
    local l_foot_pos = vector.zeros(6)
    l_foot_pos[1] = x_offset - velocity[1]*stride_cos(ph, dsp_ratio)
    l_foot_pos[2] = y_offset - velocity[2]*stride_cos(ph, dsp_ratio)
    l_foot_pos[6] = a_offset - velocity[3]*stride_cos(ph, dsp_ratio)
    if (ph < math.pi) then
      l_foot_pos[3] = z_offset + step_amplitude*step_sin(ph, dsp_ratio)
    else
      l_foot_pos[3] = z_offset
    end

    -- update right foot trajectory
    local r_foot_pos = vector.zeros(6) 
    r_foot_pos[1] = x_offset + velocity[1]*stride_cos(ph, dsp_ratio)
    r_foot_pos[2] =-y_offset + velocity[2]*stride_cos(ph, dsp_ratio)
    r_foot_pos[6] =-a_offset + velocity[3]*stride_cos(ph, dsp_ratio)
    if (ph > math.pi) then
      r_foot_pos[3] = z_offset - step_amplitude*step_sin(ph, dsp_ratio)
    else
      r_foot_pos[3] = z_offset
    end

    -- update torso trajectory
    local torso_pos = vector.zeros(6) 
    torso_pos[1] = velocity[1]*x_shift_ratio*math.sin(2*ph)
    torso_pos[2] = velocity[2]*y_shift_ratio*math.sin(2*ph)
                 - y_swing_amplitude*math.sin(ph)
    torso_pos[3] = z_swing_amplitude*-math.cos(2*ph)
    torso_pos[5] = hip_pitch_offset

    -- calculate inverse kinematics
    local qLegs = Kinematics.inverse_legs(l_foot_pos, r_foot_pos, torso_pos)

    -- add hip roll swing
    if (ph < math.pi) then 
      qLegs[2] = qLegs[2] + r_swing_amplitude*step_sin(ph, dsp_ratio)
    else
      assert(dsp_ratio)
      qLegs[8] = qLegs[8] + r_swing_amplitude*step_sin(ph, dsp_ratio)
    end

    -- write joint angles to actuator shared memory 
    actuator:set_joint_position(qLegs, 'legs')

    -- update gait parameters at the end of each cycle 
    if (ph > 2*math.pi) then
      update_gait_parameters()
      t0 = Body.get_time()
    end

    -- check for start and stop requests
    if (math.abs(math.sin(ph)) < 0.025 and stop_request) then
       active = false
       stop_request = false
       update_stance_parameters()
       t0 = Body.get_time()
    elseif start_request then
       start_request = false
    end

  else
    -- goto initial walking stance or remain stationary
    local t = Body.get_time()
    local d = util.min{(t - t0)/2, 1}
    local q = q0 + d*(vector.new(qStance) - vector.new(q0))
    actuator:set_joint_position(q, 'legs')

    -- check for start and stop requests
    if (t - t0 > 2) and start_request then
      active = true
      start_request = false
      update_gait_parameters()
      t0 = Body.get_time()
    elseif stop_request then
      stop_request = false
    end
  end
end

function walk:exit()
  self.running = false
end

return walk
