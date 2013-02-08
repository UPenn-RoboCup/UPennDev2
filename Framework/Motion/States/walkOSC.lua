--------------------------------------------------------------------------
-- Oscillatory Walking
--------------------------------------------------------------------------

require('util')
require('Platform')
require('vector')
require('Config')
require('waveform')
require('Transform')
require('Kinematics')
require('MotionState')

-- Setup 
--------------------------------------------------------------------------

walk = MotionState.new('walk')
local dcm = walk.dcm
walk:set_joint_access(0, 'all')
walk:set_joint_access(1, 'legs')

-- define velocity parameters
walk.velocity              = vector.new{0, 0, 0}
local velocity             = vector.new{0, 0, 0}
local v_limits             = vector.new(Config.walk.v_limits or {1, 1, 1})
local a_limits             = vector.new(Config.walk.a_limits or {1, 1, 1})

-- define default parameters
walk.parameters = {
  x_offset                 = 0.025, -- meters
  y_offset                 = 0.1,   -- meters
  z_offset                 = -0.75, -- meters
  a_offset                 = 0,     -- radians
  hip_pitch_offset         = 0,     -- radians
  y_offset_ratio           = 0,     -- ratio
  zmp_shift_ratio          = 0.5,   -- ratio
  y_swing_amplitude        = 0,     -- meters
  z_swing_amplitude        = 0,     -- meters
  step_amplitude           = 0,     -- meters
  period_time              = 1,     -- seconds
  dsp_ratio                = 0.25,  -- ratio
  hip_roll_fb              = 0,     -- ratio 
  knee_pitch_fb            = 0,     -- ratio
  ankle_roll_fb            = 0,     -- ratio
  ankle_pitch_fb           = 0,     -- ratio
  ankle_roll_fb_slope      = 0,     -- ratio
  ankle_pitch_fb_slope     = 0,     -- ratio
}

-- load config parameters
for k,v in pairs(walk.parameters) do
   walk.parameters[k] = Config.walk.parameters[k] or v
end

-- define local copies
local x_offset             = walk.parameters.x_offset
local y_offset             = walk.parameters.y_offset
local z_offset             = walk.parameters.z_offset
local a_offset             = walk.parameters.a_offset
local hip_pitch_offset     = walk.parameters.hip_pitch_offset
local y_offset_ratio       = walk.parameters.y_offset_ratio
local zmp_shift_ratio      = walk.parameters.zmp_shift_ratio
local y_swing_amplitude    = walk.parameters.y_swing_amplitude
local z_swing_amplitude    = walk.parameters.z_swing_amplitude
local step_amplitude       = walk.parameters.step_amplitude
local period_time          = walk.parameters.period_time
local dsp_ratio            = walk.parameters.dsp_ratio
local hip_roll_fb          = walk.parameters.hip_roll_fb
local knee_pitch_fb        = walk.parameters.knee_pitch_fb
local ankle_roll_fb        = walk.parameters.ankle_roll_fb
local ankle_pitch_fb       = walk.parameters.ankle_pitch_fb
local ankle_roll_fb_slope  = walk.parameters.ankle_roll_fb_slope
local ankle_pitch_fb_slope = walk.parameters.ankle_pitch_fb_slope

-- initialize control variables
local active               = false
local stop_request         = false
local start_request        = false
local t0                   = Platform.get_time()
local t                    = t0
local q0                   = dcm:get_joint_position_sensor('legs')
local gyro                 = vector.new{0, 0, 0}
local gyro_limits          = vector.new(Config.walk.gyro_max or {5, 5, 5})

-- Private
--------------------------------------------------------------------------

local function limit(value, threshold)
  local threshold = math.abs(threshold)
  return math.min(math.max(value, -threshold), threshold)
end

local function update_parameters()
  -- update local gait parameters
  x_offset = walk.parameters.x_offset
  y_offset = walk.parameters.y_offset
  z_offset = walk.parameters.z_offset
  a_offset = walk.parameters.a_offset
  hip_pitch_offset = walk.parameters.hip_pitch_offset
  y_offset_ratio = walk.parameters.y_offset_ratio
  zmp_shift_ratio = walk.parameters.zmp_shift_ratio
  y_swing_amplitude = walk.parameters.y_swing_amplitude
  z_swing_amplitude = walk.parameters.z_swing_amplitude
  step_amplitude = walk.parameters.step_amplitude
  period_time = walk.parameters.period_time
  dsp_ratio = walk.parameters.dsp_ratio
end

local function update_velocity(dt)
  -- update local walking velocity
  local dv = walk.velocity - velocity
  dv_limits = dt*a_limits
  dv[1] = limit(dv[1], dv_limits[1])
  dv[2] = limit(dv[2], dv_limits[2])
  dv[3] = limit(dv[3], dv_limits[3])
  velocity = velocity + dv
  velocity[1] = limit(velocity[1], v_limits[1]) 
  velocity[2] = limit(velocity[2], v_limits[2]) 
  velocity[3] = limit(velocity[3], v_limits[3]) 
end

local function update_gyro()
  -- update low pass filter for local gyro estimate 
  local beta = 0.02
  local raw_gyro = dcm:get_ahrs('gyro')
  gyro = (1 - beta)*gyro + beta*raw_gyro
  gyro[1] = limit(gyro[1], gyro_limits[1])
  gyro[2] = limit(gyro[2], gyro_limits[2])
  gyro[3] = limit(gyro[3], gyro_limits[3])
end

local function update_feedback()
  -- update feedback gains
  hip_roll_fb = math.max(walk.parameters.hip_roll_fb, 0) 
  knee_pitch_fb = math.max(walk.parameters.knee_pitch_fb, 0) 
  ankle_roll_fb = math.max(walk.parameters.ankle_roll_fb, 0) 
  ankle_pitch_fb = math.max(walk.parameters.ankle_pitch_fb, 0)
  ankle_roll_fb_slope = math.max(walk.parameters.ankle_roll_fb_slope, 0)
  ankle_pitch_fb_slope = math.max(walk.parameters.ankle_pitch_fb_slope, 0)
  ankle_roll_fb = math.max(ankle_roll_fb 
    - ankle_roll_fb*ankle_roll_fb_slope*40*math.abs(velocity[2]), 0)
  ankle_pitch_fb = math.max(ankle_pitch_fb 
    - ankle_pitch_fb*ankle_pitch_fb_slope*40*math.abs(velocity[1]), 0)
end

-- Public
--------------------------------------------------------------------------

function walk:start()
  -- issue start request
  start_request = true
end

function walk:stop()
  -- issue stop request
  stop_request = true
  self:set_velocity(0, 0, 0)
end

function walk:is_active()
  -- return true if gait is active
  return active
end

function walk:set_velocity(vx, vy, va)
  walk.velocity[1] = vx
  walk.velocity[2] = vy
  walk.velocity[3] = va 
end

function walk:get_velocity()
  return vector.copy(velocity)
end

function walk:entry()
  self.running = true
  active = false
  update_parameters()
  t0 = Platform.get_time()
  q0 = dcm:get_joint_position_sensor('legs')
  velocity = vector.new{0, 0, 0}
  dcm:set_joint_force(0, 'legs')
  dcm:set_joint_position(q0, 'legs')
  dcm:set_joint_velocity(0, 'legs')
  dcm:set_joint_stiffness(1, 'legs')
  dcm:set_joint_damping(0, 'legs')
end

function walk:update()

  -- update timing and sensor values
  ------------------------------------------------------------------------
  local dt = t - Platform.get_time()
  t = Platform.get_time()
  update_feedback()
  update_gyro()

  -- update joint positions in active gait mode
  ------------------------------------------------------------------------
  if active then
   
    -- update phase
    local ph = 2*math.pi/period_time*(t - t0)
    local step_sin = waveform.step_sin(ph, dsp_ratio)
    local stride_cos = waveform.stride_cos(ph, dsp_ratio)
    local y_stride_offset = math.abs(velocity[2])*y_offset_ratio

    -- calculate left foot trajectory
    local l_foot_pos = vector.zeros(6)
    l_foot_pos[1] = x_offset - velocity[1]*stride_cos
    l_foot_pos[2] = y_offset - velocity[2]*stride_cos + y_stride_offset
    l_foot_pos[6] = a_offset - velocity[3]*stride_cos
    if (ph < math.pi) then
      l_foot_pos[3] = z_offset + step_amplitude*step_sin
    else
      l_foot_pos[3] = z_offset
    end

    -- calculate right foot trajectory
    local r_foot_pos = vector.zeros(6) 
    r_foot_pos[1] = x_offset + velocity[1]*stride_cos
    r_foot_pos[2] =-y_offset + velocity[2]*stride_cos - y_stride_offset
    r_foot_pos[6] =-a_offset + velocity[3]*stride_cos
    if (ph > math.pi) then
      r_foot_pos[3] = z_offset - step_amplitude*step_sin
    else
      r_foot_pos[3] = z_offset
    end

    -- calculate torso trajectory
    local torso_pos = vector.zeros(6) 
    torso_pos[1] = velocity[1]*zmp_shift_ratio*math.sin(2*ph)
    torso_pos[2] = velocity[2]*zmp_shift_ratio*math.sin(2*ph)
                 - y_swing_amplitude*math.sin(ph)
    torso_pos[3] = z_swing_amplitude*-math.cos(2*ph)
    torso_pos[5] = hip_pitch_offset

    -- calculate inverse kinematics and add hip roll swing
    local p_l_foot = Transform.pose6D(l_foot_pos)
    local p_r_foot = Transform.pose6D(r_foot_pos)
    local p_torso = Transform.pose6D(torso_pos)
    local q = Kinematics.inverse_pos_legs(p_l_foot, p_r_foot, p_torso)

    -- add gyro feedback
    q[2] = q[2] - hip_roll_fb*gyro[1]
    q[4] = q[4] + knee_pitch_fb*gyro[2]
    q[5] = q[5] + ankle_pitch_fb*gyro[2]
    q[6] = q[6] + ankle_roll_fb*gyro[1]
    q[8] = q[8] - hip_roll_fb*gyro[1]
    q[10] = q[10] + knee_pitch_fb*gyro[2]
    q[11] = q[11] + ankle_pitch_fb*gyro[2]
    q[12] = q[12] + ankle_roll_fb*gyro[1]

    -- write joint angles to shared memory 
    dcm:set_joint_position(q, 'legs')

    -- update velocity during SSP 
    if (step_sin ~= 0) then
      update_velocity(dt)
    end

    -- update gait parameters at the end of each cycle 
    if (ph > 2*math.pi) then
      update_parameters()
      t0 = t
    end

    -- check for start and stop requests
    if stop_request
    and (velocity*velocity == 0)
    and (math.abs(math.sin(ph)) < 0.025) then
       active = false
       stop_request = false
       t0 = t
       q0 = dcm:get_joint_position_sensor('legs')
       velocity = vector.new{0, 0, 0}
    elseif start_request then
       start_request = false
    end

  -- update joint positions in inactive stance mode
  ------------------------------------------------------------------------
  else

    -- calculate current walking stance
    local l_foot_pos = {x_offset, y_offset, z_offset, 0, 0, a_offset}
    local r_foot_pos = {x_offset,-y_offset, z_offset, 0, 0, -a_offset}
    local torso_pos = {0, 0, 0, 0, hip_pitch_offset, 0}
    local p_l_foot = Transform.pose6D(l_foot_pos)
    local p_r_foot = Transform.pose6D(r_foot_pos)
    local p_torso = Transform.pose6D(torso_pos)
    local qstance = Kinematics.inverse_pos_legs(p_l_foot, p_r_foot, p_torso)

    -- caclutate joint configuration
    update_parameters()
    local d = util.min{(t - t0)/2, 1}
    local q = q0 + d*(vector.new(qstance) - vector.new(q0))

    -- add gyro feedback
    q[2] = q[2] - hip_roll_fb*gyro[1]
    q[4] = q[4] + knee_pitch_fb*gyro[2]
    q[5] = q[5] + ankle_pitch_fb*gyro[2]
    q[6] = q[6] + ankle_roll_fb*gyro[1]
    q[8] = q[8] - hip_roll_fb*gyro[1]
    q[10] = q[10] + knee_pitch_fb*gyro[2]
    q[11] = q[11] + ankle_pitch_fb*gyro[2]
    q[12] = q[12] + ankle_roll_fb*gyro[1]

    -- write joint angles to shared memory 
    dcm:set_joint_position(q, 'legs')

    -- check for start and stop requests
    if start_request and (t - t0 > 2) then
      active = true
      start_request = false
      t0 = t
      q0 = dcm:get_joint_position_sensor('legs')
      velocity = vector.new{0, 0, 0}
    elseif stop_request then
      stop_request = false
    end
  end
end

function walk:exit()
  self.running = false
end

return walk
