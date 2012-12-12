dofile('../include.lua')

require('Body')
require('waveform')
require('interpol')
require('Transform')
require('Kinematics')
require('vector')
require('dcm')

Body.entry()

-- initialize torso and swing foot pose relative to stance foot
local swing_pose = {-0.200, -0.194, 0.000}
local torso_pose = {-0.100, -0.097, 0.790}
local swing_goal = {
  -swing_pose[1],
   swing_pose[2],
  -swing_pose[3],
}
local torso_goal = {
  torso_pose[1] - swing_pose[1],
 -torso_pose[2] + swing_pose[2],
  torso_pose[3] - swing_pose[3],
}

-- transform left and right foot coordinates into torso base frame 
local p_l_foot = Transform.new({0, 0, 0})
local p_r_foot = Transform.new(swing_pose)
local p_torso = Transform.new(torso_pose)
p_l_foot = p_torso:inv()
p_r_foot = p_l_foot*p_r_foot

-- initialize controller parameters
local t0 = Body.get_time()
local q0 = dcm:get_joint_position('legs');
local qstance = Kinematics.inverse_pos_legs(p_l_foot, p_r_foot)

function update_stance()
  -- caclutate joint configuration
  local t = Body.get_time()
  if (t - t0 < 3) then
    local d = util.min{(t - t0)/3, 1}
    local q = q0 + d*(vector.new(qstance) - vector.new(q0))
    dcm:set_joint_position(q, 'legs')
    Body.update()
    return true
  end
end

function update_swing()
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
    local p_l_foot = Transform.new(l_foot_pos)
    local p_r_foot = Transform.new(r_foot_pos)
    local p_torso = Transform.new(torso_pos)
    local q = Kinematics.inverse_pos_legs(p_l_foot, p_r_foot, p_torso)
end

while (true) do
  update_stance()
  Body.update()
end
