--------------------------------------------------------------------------
-- Minimum Jerk Trajectory Step Controller
--------------------------------------------------------------------------

require('util')
require('Body')
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
  step_height                 = 0.036,  -- meters
  step_ds_ratio               = 0.5,    -- ratio
  x_torso_offset              = 0.0,    -- meters
  y_torso_offset              = -0.097, -- meters
  z_torso_offset              = 0.77,   -- meters
  x_torso_swing               = 0.00,   -- meters
  y_torso_swing               = 0.03,   -- meters
  z_torso_swing               = 0,      -- meters
  x_torso_swing_momentum      = 0.0,   -- meters/second
  y_torso_swing_momentum      = -0.1,   -- meters/second
  z_torso_swing_momentum      = 0,      -- meters/second
  torso_midpoint_ratio        = 0.5,    -- ratio
}

-- load config parameters
for k,v in pairs(step.parameters) do
   step.parameters[k] = Config.step.parameters[k] or v
end

-- define local copies
local step_size               = step.parameters.step_size 
local step_duration           = step.parameters.step_duration 
local step_height             = step.parameters.step_height 
local step_ds_ratio           = step.parameters.step_ds_ratio 
local x_torso_offset          = step.parameters.x_torso_offset
local y_torso_offset          = step.parameters.y_torso_offset
local z_torso_offset          = step.parameters.z_torso_offset
local x_torso_swing           = step.parameters.x_torso_swing 
local y_torso_swing           = step.parameters.y_torso_swing 
local z_torso_swing           = step.parameters.z_torso_swing 
local x_torso_swing_momentum  = step.parameters.x_torso_swing_momentum 
local y_torso_swing_momentum  = step.parameters.y_torso_swing_momentum 
local z_torso_swing_momentum  = step.parameters.z_torso_swing_momentum 
local torso_midpoint_ratio    = step.parameters.torso_midpoint_ratio 

local active                  = false
local t0                      = Body.get_time()
local q0                      = dcm:get_joint_position_sensor('legs')
local qstance                 = dcm:get_joint_position_sensor('legs')

local foot_start_t            = nil
local foot_midpoint_t         = nil
local foot_goal_t             = nil
local torso_start_t           = nil
local torso_midpoint_t        = nil
local torso_goal_t            = nil
local foot_start_position     = {}
local torso_start_position    = {}
local foot_midpoint_position  = {}
local foot_midpoint_velocity  = {}
local torso_midpoint_position = {}
local torso_midpoint_velocity = {}
local foot_goal_position      = {}
local torso_goal_position     = {}
local foot_trajectory         = {}
local torso_trajectory        = {}

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
  x_torso_swing_momentum = step.parameters.x_torso_swing_momentum 
  y_torso_swing_momentum = step.parameters.y_torso_swing_momentum 
  z_torso_swing_momentum = step.parameters.z_torso_swing_momentum 
  torso_midpoint_ratio = step.parameters.torso_midpoint_ratio 
end

local function update_trajectories()
  -- timing
  foot_start_t = step_duration*step_ds_ratio/2
  foot_midpoint_t = step_duration/2
  foot_goal_t = step_duration*(1 - step_ds_ratio/2)

  torso_start_t = 0
  torso_midpoint_t = torso_midpoint_ratio*step_duration
  torso_goal_t = step_duration

  -- start state
  foot_start_position[1] = -step_size/2
  foot_start_position[2] = 2*y_torso_offset
  foot_start_position[3] = 0

  torso_start_position[1] = x_torso_offset - step_size/4
  torso_start_position[2] = y_torso_offset
  torso_start_position[3] = z_torso_offset

  -- midpoint state
  foot_midpoint_position[1] = 0
  foot_midpoint_position[2] = 2*y_torso_offset
  foot_midpoint_position[3] = step_height

  foot_midpoint_velocity[1] = step_size/(foot_goal_t - foot_start_t)
  foot_midpoint_velocity[2] = 0
  foot_midpoint_velocity[3] = 0

  torso_midpoint_position[1] = x_torso_offset + x_torso_swing
  torso_midpoint_position[2] = y_torso_offset + y_torso_swing
  torso_midpoint_position[3] = z_torso_offset + z_torso_swing

  torso_midpoint_velocity[1] = x_torso_swing_momentum
  torso_midpoint_velocity[2] = y_torso_swing_momentum
  torso_midpoint_velocity[3] = z_torso_swing_momentum

  -- goal state
  foot_goal_position[1] = step_size/2
  foot_goal_position[2] = 2*y_torso_offset
  foot_goal_position[3] = 0

  torso_goal_position[1] = x_torso_offset + step_size/4
  torso_goal_position[2] = y_torso_offset
  torso_goal_position[3] = z_torso_offset

  -- torso and swing foot trajectories
  for i = 1,3 do
    local foot_lift_trajectory = trajectory.minimum_jerk(
      {foot_start_position[i]},
      {foot_midpoint_position[i], foot_midpoint_velocity[i]},
      foot_midpoint_t - foot_start_t)
    local foot_land_trajectory = trajectory.minimum_jerk(
      {foot_midpoint_position[i], foot_midpoint_velocity[i]},
      {foot_goal_position[i],},
      foot_goal_t - foot_midpoint_t)

    local torso_lift_trajectory = trajectory.minimum_jerk(
      {torso_start_position[i]},
      {torso_midpoint_position[i], torso_midpoint_velocity[i]},
      torso_midpoint_t - torso_start_t)
    local torso_land_trajectory = trajectory.minimum_jerk(
      {torso_midpoint_position[i], torso_midpoint_velocity[i]},
      {torso_goal_position[i]},
      torso_goal_t - torso_midpoint_t)

    foot_trajectory[i] = function (t)
      if (t < foot_midpoint_t) then
	return foot_lift_trajectory(t - foot_start_t)
      else
	return foot_land_trajectory(t - foot_midpoint_t)
      end
    end

    torso_trajectory[i] = function (t) 
      if (t < torso_midpoint_t) then
	return torso_lift_trajectory(t - torso_start_t)
      else
	return torso_land_trajectory(t - torso_midpoint_t)
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

function step:get_stance()
  return qstance
end

function step:reset()
  t0 = Body.get_time()
  update_parameters()
  update_trajectories()
  -- initialize stance 
  local p_l_foot = Transform.pose6D({0, 0, 0})
  local p_r_foot = Transform.pose6D(foot_start_position)
  local p_torso = Transform.pose6D(torso_start_position)
  p_l_foot = p_torso:inv()
  p_r_foot = p_l_foot*p_r_foot
  qstance = Kinematics.inverse_pos_legs(p_l_foot, p_r_foot)
end

function step:entry()
  q0 = dcm:get_joint_position_sensor('legs')
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
    for i = 1,3 do
      torso_position[i] = torso_trajectory[i](t)
      foot_position[i] = foot_trajectory[i](t)
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
  end
end

function step:exit()
  active = false
end

return step
