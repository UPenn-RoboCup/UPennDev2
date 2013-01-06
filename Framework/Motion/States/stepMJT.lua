--------------------------------------------------------------------------
-- Minimum Jerk Trajectory Step Controller
--------------------------------------------------------------------------

require('mcm')
require('util')
require('Body')
require('vector')
require('Config')
require('trajectory')
require('Kinematics')
require('MotionState')

-- Setup 
--------------------------------------------------------------------------

step = MotionState.new(...)
local dcm = step.dcm
step:set_joint_access(0, 'all')
step:set_joint_access(1, 'legs')

-- define default parameters
step.parameters = {
  step_size                   = 0.2,    -- meters
  step_duration               = 1.0,    -- seconds
  step_height                 = 0.04,  -- meters
  step_ds_ratio               = 0.4,    -- ratio
  x_torso_offset              = 0.0,    -- meters
  y_torso_offset              = -0.097, -- meters
  z_torso_offset              = 0.77,   -- meters
  x_torso_swing               = 0.00,   -- meters
  y_torso_swing               = 0.063,  -- meters
  z_torso_swing               = 0,      -- meters
  x_torso_start_velocity      = 0.12,   -- meters/second
  y_torso_start_velocity      = 0.20,   -- meters/second
  z_torso_start_velocity      = 0.0,    -- meters/second
  x_torso_midpoint_velocity   = 0.05,   -- meters/second
  y_torso_midpoint_velocity   = 0.0,    -- meters/second
  z_torso_midpoint_velocity   = 0.0,    -- meters/second
}

-- load config parameters
for k,v in pairs(step.parameters) do
   step.parameters[k] = Config.step.parameters[k] or v
end

-- define local copies
local step_size                 = step.parameters.step_size 
local step_duration             = step.parameters.step_duration 
local step_height               = step.parameters.step_height 
local step_ds_ratio             = step.parameters.step_ds_ratio 
local x_torso_offset            = step.parameters.x_torso_offset
local y_torso_offset            = step.parameters.y_torso_offset
local z_torso_offset            = step.parameters.z_torso_offset
local x_torso_swing             = step.parameters.x_torso_swing 
local y_torso_swing             = step.parameters.y_torso_swing 
local z_torso_swing             = step.parameters.z_torso_swing 
local x_torso_start_velocity    = step.parameters.x_torso_start_velocity
local y_torso_start_velocity    = step.parameters.y_torso_start_velocity
local z_torso_start_velocity    = step.parameters.z_torso_start_velocity
local x_torso_midpoint_velocity = step.parameters.x_torso_midpoint_velocity
local y_torso_midpoint_velocity = step.parameters.y_torso_midpoint_velocity
local z_torso_midpoint_velocity = step.parameters.z_torso_midpoint_velocity

local active                  = false
local t0                      = Body.get_time()
local ssp_begin_t             = nil
local ssp_end_t               = nil

local foot_start_position     = {}
local foot_start_velocity     = {}
local foot_midpoint_position  = {}
local foot_midpoint_velocity  = {}
local foot_goal_position      = {}
local foot_goal_velocity      = {}
local foot_trajectory         = {}

local torso_start_position    = {}
local torso_start_velocity    = {}
local torso_midpoint_position = {}
local torso_midpoint_velocity = {}
local torso_goal_position     = {}
local torso_goal_velocity     = {}
local torso_trajectory        = {}

local cop_start_position      = {}
local cop_start_velocity      = {}
local cop_ssp_begin_position  = {}
local cop_ssp_begin_velocity  = {}
local cop_ssp_end_position    = {}
local cop_ssp_end_velocity    = {}
local cop_goal_position       = {}
local cop_goal_velocity       = {}
local cop_trajectory          = {}

local l_foot_sole_transform   = Config.mechanics.l_foot.sole_transform
local r_foot_sole_transform   = Config.mechanics.r_foot.sole_transform
local l_foot_sole_offset      = l_foot_sole_transform:get_pose6D()
local r_foot_sole_offset      = r_foot_sole_transform:get_pose6D()

-- Private
--------------------------------------------------------------------------

local function update_parameters()
  -- update local gait parameters
  step_size = step.parameters.step_size 
  step_duration = step.parameters.step_duration 
  step_height = step.parameters.step_height 
  step_ds_ratio = step.parameters.step_ds_ratio 
  x_torso_offset = step.parameters.x_torso_offset 
  y_torso_offset = step.parameters.y_torso_offset 
  z_torso_offset = step.parameters.z_torso_offset 
  x_torso_swing = step.parameters.x_torso_swing 
  y_torso_swing = step.parameters.y_torso_swing 
  z_torso_swing = step.parameters.z_torso_swing 
  x_torso_start_velocity = step.parameters.x_torso_start_velocity
  y_torso_start_velocity = step.parameters.y_torso_start_velocity
  z_torso_start_velocity = step.parameters.z_torso_start_velocity
  x_torso_midpoint_velocity = step.parameters.x_torso_midpoint_velocity
  y_torso_midpoint_velocity = step.parameters.y_torso_midpoint_velocity
  z_torso_midpoint_velocity = step.parameters.z_torso_midpoint_velocity
end

local function update_trajectories()
  -- timing
  ssp_begin_t = step_duration*step_ds_ratio/2
  ssp_end_t = step_duration*(1 - step_ds_ratio/2)

  -- foot state
  foot_start_position[1] = -step_size/2
  foot_start_position[2] = 2*y_torso_offset
  foot_start_position[3] = 0
  foot_start_velocity[1] = 0 
  foot_start_velocity[2] = 0 
  foot_start_velocity[3] = 0

  foot_midpoint_position[1] = 0
  foot_midpoint_position[2] = 2*y_torso_offset
  foot_midpoint_position[3] = step_height
  foot_midpoint_velocity[1] = step_size/(ssp_end_t - ssp_begin_t)
  foot_midpoint_velocity[2] = 0
  foot_midpoint_velocity[3] = 0

  foot_goal_position[1] = step_size/2
  foot_goal_position[2] = 2*y_torso_offset
  foot_goal_position[3] = 0
  foot_goal_velocity[1] = 0 
  foot_goal_velocity[2] = 0
  foot_goal_velocity[3] = 0

  -- torso state
  torso_start_position[1] = x_torso_offset - step_size/4
  torso_start_position[2] = y_torso_offset
  torso_start_position[3] = z_torso_offset
  torso_start_velocity[1] = x_torso_start_velocity
  torso_start_velocity[2] = y_torso_start_velocity
  torso_start_velocity[3] = z_torso_start_velocity

  torso_midpoint_position[1] = x_torso_offset + x_torso_swing
  torso_midpoint_position[2] = y_torso_offset + y_torso_swing
  torso_midpoint_position[3] = z_torso_offset + z_torso_swing
  torso_midpoint_velocity[1] = x_torso_midpoint_velocity
  torso_midpoint_velocity[2] = y_torso_midpoint_velocity
  torso_midpoint_velocity[3] = z_torso_midpoint_velocity

  torso_goal_position[1] = x_torso_offset + step_size/4
  torso_goal_position[2] = y_torso_offset
  torso_goal_position[3] = z_torso_offset
  torso_goal_velocity[1] = x_torso_start_velocity 
  torso_goal_velocity[2] = -y_torso_start_velocity
  torso_goal_velocity[3] = z_torso_start_velocity

  -- cop state
  cop_start_position[1] = -step_size/2 + r_foot_sole_offset[1]
  cop_start_position[2] = 2*y_torso_offset + r_foot_sole_offset[2]
  cop_start_position[3] = 0 
  cop_start_velocity[1] = 0
  cop_start_velocity[2] = 0
  cop_start_velocity[3] = 0

  cop_ssp_begin_position[1] = l_foot_sole_offset[1] 
  cop_ssp_begin_position[2] = l_foot_sole_offset[2] 
  cop_ssp_begin_position[3] = 0
  cop_ssp_begin_velocity[1] = 0
  cop_ssp_begin_velocity[2] = 0
  cop_ssp_begin_velocity[3] = 0

  cop_ssp_end_position[1] = l_foot_sole_offset[1]
  cop_ssp_end_position[2] = l_foot_sole_offset[2]
  cop_ssp_end_position[3] = 0
  cop_ssp_end_velocity[1] = 0
  cop_ssp_end_velocity[2] = 0
  cop_ssp_end_velocity[3] = 0

  cop_goal_position[1] = step_size/2 + r_foot_sole_offset[1]
  cop_goal_position[2] = 2*y_torso_offset + r_foot_sole_offset[2]
  cop_goal_position[3] = 0
  cop_goal_velocity[1] = 0
  cop_goal_velocity[2] = 0
  cop_goal_velocity[3] = 0

  -- torso and swing foot trajectories
  for i = 1,3 do
    local foot_lift_trajectory = trajectory.minimum_jerk(
      {foot_start_position[i], foot_start_velocity[i]},
      {foot_midpoint_position[i], foot_midpoint_velocity[i]},
      (ssp_end_t- ssp_begin_t)/2)
    local foot_land_trajectory = trajectory.minimum_jerk(
      {foot_midpoint_position[i], foot_midpoint_velocity[i]},
      {foot_goal_position[i], foot_goal_velocity[i]},
      (ssp_end_t - ssp_begin_t)/2)

    local torso_lift_trajectory = trajectory.minimum_jerk(
      {torso_start_position[i], torso_start_velocity[i]},
      {torso_midpoint_position[i], torso_midpoint_velocity[i]},
      step_duration/2)
    local torso_land_trajectory = trajectory.minimum_jerk(
      {torso_midpoint_position[i], torso_midpoint_velocity[i]},
      {torso_goal_position[i], torso_goal_velocity[i]},
      step_duration/2)

    local cop_lift_trajectory = trajectory.minimum_jerk(
      {cop_start_position[i], cop_start_velocity[i]},
      {cop_ssp_begin_position[i], cop_ssp_begin_velocity[i]},
      ssp_begin_t*2)
    local cop_ssp_trajectory = trajectory.minimum_jerk(
      {cop_ssp_begin_position[i], cop_ssp_begin_velocity[i]},
      {cop_ssp_end_position[i], cop_ssp_end_velocity[i]},
      (ssp_end_t - ssp_begin_t))
    local cop_land_trajectory = trajectory.minimum_jerk(
      {cop_ssp_end_position[i], cop_ssp_end_velocity[i]},
      {cop_goal_position[i], cop_goal_velocity[i]},
      ssp_begin_t*2)

    foot_trajectory[i] = function (t)
      if (t < step_duration/2) then
	return foot_lift_trajectory(t - ssp_begin_t)
      else
	return foot_land_trajectory(t - step_duration/2)
      end
    end

    torso_trajectory[i] = function (t) 
      if (t < step_duration/2) then
	return torso_lift_trajectory(t)
      else
	return torso_land_trajectory(t - step_duration/2)
      end
    end

    cop_trajectory[i] = function (t) 
      if (t < ssp_begin_t) then
        return cop_lift_trajectory(t + ssp_begin_t)
      elseif (t > ssp_end_t) then
        return cop_land_trajectory(t - ssp_end_t)
      else
        return cop_ssp_trajectory(t - ssp_begin_t)
      end
    end

  end
end


-- Public
--------------------------------------------------------------------------

function step:is_active()
  return active
end

function step:get_torso_trajectory()
  return torso_trajectory
end

function step:get_swing_foot_trajectory()
  return foot_trajectory
end

function step:get_cop_trajectory()
  return cop_trajectory
end

function step:get_torso_start_position()
  return torso_start_position
end

function step:get_torso_start_velocity()
  return torso_start_velocity
end

function step:get_joint_start_position()
  -- initialize stance 
  local p_l_foot = Transform.pose6D({0, 0, 0})
  local p_r_foot = Transform.pose6D(foot_start_position)
  local p_torso = Transform.pose6D(torso_start_position)
  p_l_foot = p_torso:inv()
  p_r_foot = p_l_foot*p_r_foot
  return Kinematics.inverse_pos_legs(p_l_foot, p_r_foot)
end

function step:get_desired_cop()
  return desired_cop
end

function step:reset()
  t0 = Body.get_time()
  update_parameters()
  update_trajectories()
end

function step:entry()
  local q0 = dcm:get_joint_position_sensor('legs')
  dcm:set_joint_force(0, 'legs')
  dcm:set_joint_position(q0, 'legs')
  dcm:set_joint_velocity(0, 'legs')
  dcm:set_joint_stiffness(1, 'legs')
  dcm:set_joint_damping(0, 'legs')
  active = true
  self:reset()
end

function step:update()
  local t = Body.get_time() - t0
  if (t < step_duration) then
    -- calculate swing foot and torso coordinates relative stance foot 
    local foot_position = {}
    local torso_position = {}
    local desired_cop = {}
    for i = 1,3 do
      torso_position[i] = torso_trajectory[i](t)
      foot_position[i] = foot_trajectory[i](t)
      desired_cop[i] = cop_trajectory[i](t) - torso_position[i]
    end

    -- transform left and right foot coordinates into torso base frame 
    local p_l_foot = Transform.pose6D({0, 0, 0})
    local p_r_foot = Transform.pose6D(foot_position)
    local p_torso = Transform.pose6D(torso_position)
    p_l_foot = p_torso:inv()
    p_r_foot = p_l_foot*p_r_foot

    -- update actuators
    local q = Kinematics.inverse_pos_legs(p_l_foot, p_r_foot)
    dcm:set_joint_position(q, 'legs')

    -- update desired center of pressure
    mcm:set_desired_cop(desired_cop)
  else
    active = false
  end
end

function step:exit()
  active = false
end

return step
