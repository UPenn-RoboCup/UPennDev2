dofile('../include.lua')

require('Body')
require('trajectory')
require('Transform')
require('Kinematics')
require('vector')
require('dcm')

Body.entry()

-- initialize step parameters (coordinates relative to l_foot frame)
----------------------------------------------------------------------

local step_duration = 0.6
local step_size = 0.35
local step_height = 0.05

local x_offset = 0
local y_offset = -0.097
local z_offset = 0.77

local x_torso_swing = 0.05
local y_torso_swing = -y_offset
local z_torso_swing = 0
local x_torso_swing_momentum = -0.2
local y_torso_swing_momentum = 0.2
local z_torso_swing_momentum = 0

local torso_midpoint_ratio = 0.5
local swing_midpoint_ratio = 0.5

-- start conditions
local swing_start_position = {
  -step_size/2,
  2*y_offset,
  0,
}
local torso_start_position = {
  0, 
  0,
  z_offset,
}
local swing_start_velocity = {
  0,
  0,
  0,
}
local torso_start_velocity = {
  0,
  0,
  0,
}

-- midpoint conditions
local torso_midpoint_t = torso_midpoint_ratio*step_duration
local swing_midpoint_t = swing_midpoint_ratio*step_duration
local torso_midpoint_position = {
  x_offset + x_torso_swing,
  y_offset + y_torso_swing,
  z_offset + z_torso_swing,
}
local swing_midpoint_position = {
  0,
  2*y_offset,
  step_height,
} 
local torso_midpoint_velocity = {
  x_torso_swing_momentum,
  y_torso_swing_momentum,
  z_torso_swing_momentum,
}
local swing_midpoint_velocity = {
  step_size/step_duration,
  0,
  0,
}

-- goal conditions
local swing_goal_t = step_duration
local torso_goal_t = step_duration
local swing_goal_position = {
  step_size/2,
  2*y_offset,
  0,
}
local torso_goal_position = {
  x_offset + step_size/4,
  1.5*y_offset,
  z_offset,
}
local swing_goal_velocity = {
  0, 
  0, 
  0, 
}
local torso_goal_velocity = {
  0,
  0,
  0,
}

-- initialize controller parameters
----------------------------------------------------------------------

local t0 = Body.get_time()
local q0 = dcm:get_joint_position('legs');

local torso_lift_trajectory = {}
local torso_land_trajectory = {}
local swing_lift_trajectory = {}
local swing_land_trajectory = {}

for i = 1,3 do
  torso_lift_trajectory[i] = trajectory.minimum_jerk(
    {torso_start_position[i], torso_start_velocity[i]},
    {torso_midpoint_position[i], torso_midpoint_velocity[i]},
    torso_midpoint_t)
  torso_land_trajectory[i] = trajectory.minimum_jerk(
    {torso_midpoint_position[i], torso_midpoint_velocity[i]},
    {torso_goal_position[i], torso_goal_velocity[i]},
    torso_goal_t - torso_midpoint_t)
  swing_lift_trajectory[i] = trajectory.minimum_jerk(
    {swing_start_position[i], swing_start_velocity[i]},
    {swing_midpoint_position[i], swing_midpoint_velocity[i]},
    swing_midpoint_t)
  swing_land_trajectory[i] = trajectory.minimum_jerk(
    {swing_midpoint_position[i], swing_midpoint_velocity[i]},
    {swing_goal_position[i], swing_goal_velocity[i]},
    swing_goal_t - swing_midpoint_t)
end

-- define step states
----------------------------------------------------------------------

function stance_update()
  -- caclutate stance configuration
  local t = Body.get_time()
  if (t - t0 < 1) then
    -- transform left and right foot coordinates into torso base frame 
    local p_l_foot = Transform.pose6D({0, 0, 0})
    local p_r_foot = Transform.pose6D(swing_start_position)
    local p_torso = Transform.pose6D(torso_start_position)
    p_l_foot = p_torso:inv()
    p_r_foot = p_l_foot*p_r_foot
    local qstance = Kinematics.inverse_pos_legs(p_l_foot, p_r_foot)

    -- update actuators
    local d = util.min{(t - t0)/1, 1}
    local q = q0 + d*(vector.pose6D(qstance) - vector.pose6D(q0))
    dcm:set_joint_position(q, 'legs')
    return true
  end
end

function step_update()
  -- calculate step configuration
  local t = Body.get_time()
  if ((t - t0) < step_duration) then
    local torso_position = {}
    local swing_position = {}
    for i = 1,3 do
      if ((t - t0) < torso_midpoint_t) then
	torso_position[i] = torso_lift_trajectory[i](t - t0)
      else
	torso_position[i] = torso_land_trajectory[i](t - t0 - torso_midpoint_t)
      end
      if ((t - t0) < swing_midpoint_t) then
	swing_position[i] = swing_lift_trajectory[i](t - t0)
      else
	swing_position[i] = swing_land_trajectory[i](t - t0 - swing_midpoint_t)
      end
    end

    -- transform left and right foot coordinates into torso base frame 
    local p_l_foot = Transform.pose6D({0, 0, 0})
    local p_r_foot = Transform.pose6D(swing_position)
    local p_torso = Transform.pose6D(torso_position)
    p_l_foot = p_torso:inv()
    p_r_foot = p_l_foot*p_r_foot

    -- update actuators
    local q = Kinematics.inverse_pos_legs(p_l_foot, p_r_foot)
    dcm:set_joint_position(q, 'legs')
    return true
  end
end


print('stance update')
t0 = Body.get_time()
while (stance_update()) do
  Body.set_simulator_pose({0, 0, 0.6, 0})
  Body.update()
end
Body.set_simulator_pose({0, 0, 0.577, 0})

print('stabilizing')
t0 = Body.get_time()
while (Body.get_time() - t0 < 2) do
  Body.update()
end

print('step update')
t0 = Body.get_time()
while (true) do
  Body.update()
  step_update()
end
