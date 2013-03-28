require('util')
require('Platform')
require('vector')
require('Config')
require('waveform')
require('Transform')
require('Kinematics')
require('Motion_state')

--------------------------------------------------------------------------
-- Stand Controller
--------------------------------------------------------------------------

stand = Motion_state.new('stand')
stand:set_joint_access(0, 'all')
stand:set_joint_access(1, 'lowerbody')
local dcm = stand.dcm

-- default parameters
stand.parameters = {
  x_offset                 = 0.025, -- meters
  y_offset                 = 0.1,   -- meters
  z_offset                 = -0.75, -- meters
  a_offset                 = 0,     -- radians
  hip_pitch_offset         = 0,     -- radians
  hip_roll_fb              = 0,     -- ratio 
  knee_pitch_fb            = 0,     -- ratio
  ankle_roll_fb            = 0,     -- ratio
  ankle_pitch_fb           = 0,     -- ratio
}

-- config parameters
stand:load_parameters()

-- define local copies
local x_offset             = stand.parameters.x_offset
local y_offset             = stand.parameters.y_offset
local z_offset             = stand.parameters.z_offset
local a_offset             = stand.parameters.a_offset
local hip_pitch_offset     = stand.parameters.hip_pitch_offset
local hip_roll_fb          = stand.parameters.hip_roll_fb
local knee_pitch_fb        = stand.parameters.knee_pitch_fb
local ankle_roll_fb        = stand.parameters.ankle_roll_fb
local ankle_pitch_fb       = stand.parameters.ankle_pitch_fb

-- Private
--------------------------------------------------------------------------

local gyro                 = vector.new{0, 0, 0}
local gyro_limits          = vector.new{5, 5, 5}
local q0                   = dcm:get_joint_position_sensor('legs')
local t0                   = Platform.get_time()
local t                    = t0

local function limit(value, threshold)
  local threshold = math.abs(threshold)
  return math.min(math.max(value, -threshold), threshold)
end

local function update_stance_parameters()
  -- update local stance parameters
  x_offset = stand.parameters.x_offset
  y_offset = stand.parameters.y_offset
  z_offset = stand.parameters.z_offset
  a_offset = stand.parameters.a_offset
  hip_pitch_offset = stand.parameters.hip_pitch_offset
  hip_roll_fb = stand.parameters.hip_roll_fb
  knee_pitch_fb = stand.parameters.knee_pitch_fb
  ankle_roll_fb = stand.parameters.ankle_roll_fb
  ankle_pitch_fb = stand.parameters.ankle_pitch_fb
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

-- Public
--------------------------------------------------------------------------

function stand:entry()
  t  = Platform.get_time()
  t0 = Platform.get_time()
  q0 = dcm:get_joint_position_sensor('lowerbody')
  dcm:set_joint_force(0, 'lowerbody')
  dcm:set_joint_position(q0, 'lowerbody')
  dcm:set_joint_velocity(0, 'lowerbody')
  dcm:set_joint_p_gain(1, 'lowerbody')
  dcm:set_joint_i_gain(0.1, 'lowerbody')
  dcm:set_joint_d_gain(0.01, 'lowerbody')
  update_stance_parameters()
end

function stand:update()
  local dt = t - Platform.get_time()
  t = Platform.get_time()
  update_gyro()
  update_stance_parameters()

  -- calculate current walking stance
  local l_foot_pos = {x_offset, y_offset, z_offset, 0, 0, a_offset}
  local r_foot_pos = {x_offset,-y_offset, z_offset, 0, 0, -a_offset}
  local torso_pos = {0, 0, 0, 0, hip_pitch_offset, 0}
  local p_l_foot = Transform.pose(l_foot_pos)
  local p_r_foot = Transform.pose(r_foot_pos)
  local p_torso = Transform.pose(torso_pos)
  local q_stance = Kinematics.inverse_pos_legs(p_l_foot, p_r_foot, p_torso)

  -- calculate joint configuration
  local d = util.min{(t - t0)/2, 1}
  local q = q0 + d*(vector.new(q_stance) - vector.new(q0))

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
end

function stand:exit()
end

return stand
