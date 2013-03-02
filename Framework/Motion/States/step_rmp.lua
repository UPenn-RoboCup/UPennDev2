require('Platform')
require('Config')
require('Kinematics')
require('Motion_state')
require('trajectory')
require('vector')
require('mcm')
require('pcm')
require('rmp')

--------------------------------------------------------------------------------
-- Rhythmic Movement Primitive Step Controller
--------------------------------------------------------------------------------

-- Setup 
--------------------------------------------------------------------------------

step = Motion_state.new(...)
local dcm = step.dcm
step:set_joint_access(0, 'all')
step:set_joint_access(1, 'legs')

local torso_rmp = rmp.new(
  3,                                       -- number of rmp dimensions
  30,                                      -- number of basis functions
  {'periodic', 'antiperiodic', 'periodic'} -- type of basis functions
)

-- Utilities
--------------------------------------------------------------------------------

local function zeros(n)
  local t = {}
  for i = 1,n do
    t[i] = 0
  end
  return t
end

local function ones(n)
  local t = {}
  for i = 1,n do
    t[i] = 1
  end
  return t
end

local function copy_array(ts, td)
  if not ts then return nil end
  local td = td or {}
  for i = 1,#ts do
    td[i] = ts[i]
  end
  return td
end

-- Parameters
--------------------------------------------------------------------------------
step.parameters = {
  step_duration               = 0.70,   -- seconds
  step_height                 = 0.03,   -- meters
  step_ds_ratio               = 0.40,   -- ratio
  x_foot_offset               = 0.00,   -- meters
  y_foot_offset               = 0.097,  -- meters
  z_foot_offset               = -0.77,  -- meters
  rmp_parameters              = torso_rmp:get_parameters(),
  nominal_rmp_state           = torso_rmp:get_state(),
}

-- Private
--------------------------------------------------------------------------------

-- define config variables
local sole_dimensions            = Config.mechanics.l_foot.sole_dimensions
local l_foot_sole_transform      = Config.mechanics.l_foot.sole_transform
local r_foot_sole_transform      = Config.mechanics.r_foot.sole_transform
local l_foot_sole_offset         = l_foot_sole_transform:get_pose()
local r_foot_sole_offset         = r_foot_sole_transform:get_pose()

-- define step variables
local active                     = false
local velocity                   = zeros(3)
local support_foot               = 'l'
local nominal_initialization     = false
local t0                         = nil
local ss_begin_t                 = nil 
local ss_end_t                   = nil 
local swing_foot_offset          = nil
local swing_foot_sole_offset     = nil 
local swing_foot_start_pose      = nil
local swing_foot_start_twist     = nil
local support_foot_offset        = nil
local support_foot_sole_offset   = nil 
local support_foot_start_pose    = nil
local support_foot_start_twist   = nil

-- define control variables
local swing_foot_state           = {zeros(3), zeros(3), zeros(3)}
local swing_foot_via_state       = {zeros(3), zeros(3), zeros(3)}
local swing_foot_goal_state      = {zeros(3), zeros(3), zeros(3)}
local torso_state                = {zeros(3), zeros(3), zeros(3)}
local torso_reference_state      = {zeros(3), zeros(3), zeros(3)}
local torso_reference_trajectory = nil

-- define local parameters
local step_duration           = step.parameters.step_duration 
local step_height             = step.parameters.step_height 
local step_ds_ratio           = step.parameters.step_ds_ratio 
local x_foot_offset           = step.parameters.x_foot_offset
local y_foot_offset           = step.parameters.y_foot_offset
local z_foot_offset           = step.parameters.z_foot_offset
local rmp_parameters          = step.parameters.rmp_parameters
local nominal_rmp_state       = step.parameters.nominal_rmp_state

local function update_parameters()
  step_duration = step.parameters.step_duration 
  step_height = step.parameters.step_height 
  step_ds_ratio = step.parameters.step_ds_ratio 
  x_foot_offset = step.parameters.x_foot_offset
  y_foot_offset = step.parameters.y_foot_offset
  z_foot_offset = step.parameters.z_foot_offset
  rmp_parameters = step.parameters.rmp_parameters
  nominal_rmp_state = step.parameters.nominal_rmp_state
end

local function initialize_step_variables()

  t0 = Platform.get_time()
  ss_begin_t = step_duration*(step_ds_ratio/2)
  ss_end_t = step_duration*(1 - step_ds_ratio/2)

  local l_foot_offset = {x_foot_offset, y_foot_offset, z_foot_offset}
  local r_foot_offset = {x_foot_offset,-y_foot_offset, z_foot_offset}

  if (support_foot == 'r') then
    swing_foot_offset = l_foot_offset
    swing_foot_sole_offset = l_foot_sole_offset
    swing_foot_start_pose = pcm:get_l_foot_pose() 
    swing_foot_start_twist = pcm:get_l_foot_twist() 

    support_foot_offset = r_foot_offset
    support_foot_sole_offset = r_foot_sole_offset
    support_foot_start_pose = pcm:get_r_foot_pose() 
    support_foot_start_twist = pcm:get_r_foot_twist() 
  else
    swing_foot_offset = r_foot_offset
    swing_foot_sole_offset = r_foot_sole_offset
    swing_foot_start_pose = pcm:get_r_foot_pose() 
    swing_foot_start_twist = pcm:get_r_foot_twist() 

    support_foot_offset = l_foot_offset
    support_foot_sole_offset = l_foot_sole_offset
    support_foot_start_pose = pcm:get_l_foot_pose() 
    support_foot_start_twist = pcm:get_l_foot_twist() 
  end
end

local function initialize_torso_variables()

  -- intialize torso reference trajectory
  torso_reference_trajectory = function(t)
    local reference_position = {}
    for i = 1, 3 do
      reference_position[i] = velocity[i]*(t - step_duration/2)
                            - support_foot_offset[i]
    end
    return reference_position
  end

  -- intialize torso reference state
  local reference_start_position = torso_reference_trajectory(0)
  local reference_start_velocity = velocity

  -- initialize torso rmp
  torso_rmp:initialize({zeros(3)}, zeros(3), ones(3), step_duration)
  torso_rmp:set_parameters(rmp_parameters)
  torso_rmp:set_time_step(Platform.get_time_step())
  if (support_foot == 'r') then
    -- TEMPORARY HACK (FOR FORWARD/VERTICAL STEPPING ONLY !)
    torso_rmp:set_phase(2*math.pi)
  end
  
  -- intialize torso rmp state
  if (nominal_initialization) then
    -- TEMPORARY HACK (FOR FORWARD/VERTICAL STEPPING ONLY !)
    local rmp_state = {{}, {}, {}} 
    if (support_foot == 'r') then
      for i = 1, 3 do
        rmp_state[1][i] = nominal_rmp_state[1][i]
        rmp_state[2][i] =-nominal_rmp_state[2][i]
        rmp_state[3][i] = nominal_rmp_state[3][i]
      end
    else
      for i = 1, 3 do
        rmp_state[1][i] = nominal_rmp_state[1][i]
        rmp_state[2][i] = nominal_rmp_state[2][i]
        rmp_state[3][i] = nominal_rmp_state[3][i]
      end
    end
    torso_rmp:set_state(rmp_state)
  else
    local rmp_state = {{}, {}, {}}
    for i = 1, 3 do
      -- TEMPORARY HACK (WHY DOES SUPPORT FOOT TWIST HAVE OPPOSITE SIGN?)
      rmp_state[i][1] =-support_foot_start_pose[i] - reference_start_position[i]
      rmp_state[i][2] = support_foot_start_twist[i] - reference_start_velocity[i]
      rmp_state[i][3] = 0
    end
    torso_rmp:set_state(rmp_state)
  end

  -- intialize torso state
  for i = 1, 3 do
    torso_reference_state[1][i] = reference_start_position[i]
    torso_reference_state[2][i] = reference_start_velocity[i]
    torso_reference_state[3][i] = 0
    torso_state[1][i] = reference_start_position[i] + torso_rmp:get_position(i)
    torso_state[2][i] = reference_start_velocity[i] + torso_rmp:get_velocity(i) 
    torso_state[3][i] = torso_rmp:get_acceleration(i)
  end
end

local function initialize_swing_foot_variables()

  local reference_prev_position = torso_reference_trajectory(-step_duration/2)
  local reference_next_position = torso_reference_trajectory(3*step_duration/2)

  -- initialize swing foot state
  if (nominal_initialization) then
    for i = 1, 3 do
      swing_foot_state[1][i] = reference_prev_position[i] + swing_foot_offset[i]
      swing_foot_state[2][i] = 0
      swing_foot_state[3][i] = 0
    end
  else
    for i = 1, 3 do
      swing_foot_state[1][i] = swing_foot_start_pose[i] - support_foot_start_pose[i]
      swing_foot_state[2][i] = 0
      swing_foot_state[3][i] = 0
    end
  end

  -- initialize goal state
  for i = 1, 3 do
    swing_foot_goal_state[1][i] = reference_next_position[i] + swing_foot_offset[i]
    swing_foot_goal_state[2][i] = 0
    swing_foot_goal_state[3][i] = 0
  end

  -- initialize via state
  for i = 1, 3 do
    swing_foot_via_state[1][i] = 0.5*(swing_foot_state[1][i]
                               + swing_foot_goal_state[1][i])
    swing_foot_via_state[2][i] = (swing_foot_goal_state[1][i]
                               - swing_foot_state[1][i])/(ss_end_t - ss_begin_t)
    swing_foot_via_state[3][i] = 0
  end
  swing_foot_via_state[1][3] = swing_foot_via_state[1][3] + step_height
end

local function update_torso_state(t, dt)

  local reference_position = torso_reference_trajectory(t)
  local reference_velocity = velocity

  -- update torso state via rhythmic movement primitive
  torso_rmp:set_time_step(dt)
  torso_rmp:integrate()

  for i = 1, 3 do
    torso_reference_state[1][i] = reference_position[i]
    torso_reference_state[2][i] = reference_velocity[i]
    torso_reference_state[3][i] = 0
    torso_state[1][i] = reference_position[i] + torso_rmp:get_position(i)
    torso_state[2][i] = reference_velocity[i] + torso_rmp:get_velocity(i) 
    torso_state[3][i] = torso_rmp:get_acceleration(i)
  end
end

local function update_swing_foot_state(t, dt)

  local duration, desired_state

  if (t < ss_begin_t) then
    -- swing foot remains stationary relative to support foot 
    return
  elseif (t < step_duration/2) then
    -- swing foot moves toward via point
    duration = step_duration/2 - t
    desired_state = swing_foot_via_state
  elseif (t < ss_end_t) then
    -- swing foot moves toward goal position
    duration = ss_end_t - t
    desired_state = swing_foot_goal_state
  else
    -- swing foot remains stationary relative to support foot 
    return
  end

  -- update swing foot state via minimum jerk trajectory
  for i = 1, 3 do
    local s0 = {
      swing_foot_state[1][i],
      swing_foot_state[2][i],
      swing_foot_state[3][i],
    }
    local s1 = {
      desired_state[1][i],
      desired_state[2][i],
      desired_state[3][i],
    }
    local s = {trajectory.minimum_jerk_step(s0, s1, duration, dt)}
    swing_foot_state[1][i] = s[1]
    swing_foot_state[2][i] = s[2]
    swing_foot_state[3][i] = s[3]
  end
end

local function update_desired_cop(t, dt)

  local support_foot_position, swing_foot_position
  if (support_foot == 'r') then
    support_foot_position = pcm:get_r_foot_pose()
    swing_foot_position = pcm:get_l_foot_pose()
  else
    support_foot_position = pcm:get_l_foot_pose()
    swing_foot_position = pcm:get_r_foot_pose()
  end

  local support_sole_position = {}
  local swing_sole_position = {}
  for i = 1, 2 do
    support_sole_position[i] = support_foot_position[i]
                             + support_foot_sole_offset[i]
    swing_sole_position[i] = swing_foot_position[i]
                           + swing_foot_sole_offset[i]
  end

  local tipping_eps = 5e-3
  local tipping_status = 0
  local actual_cop = pcm:get_cop()
  local desired_cop = zeros(3)
  local cop_error = zeros(3)

  if (t > ss_begin_t) and (t < ss_end_t) then
    -- desired CoP is the center of the support foot during single support

    -- calculate desired CoP
    for i = 1, 2 do
      desired_cop[i] = support_sole_position[i]
      cop_error[i] = actual_cop[i] - desired_cop[i] 
    end

    -- determine tipping status
    for i = 1, 2 do
      if (math.abs(cop_error[i]) > sole_dimensions[i]/2 - tipping_eps) then
        tipping_status = 1
      end
    end
  else
    -- desired CoP is the projection of the actual CoP onto the line segment 
    -- connecting the center of each foot during double support

    -- calculate desired CoP
    local cop_vector = {}
    local support_vector = {} 
    for i = 1, 2 do
      support_vector[i] = swing_sole_position[i] - support_sole_position[i]
      cop_vector[i] = actual_cop[i] - support_sole_position[i]
    end

    local support_norm = support_vector[1]^2 + support_vector[2]^2
    local cop_projection = 0 
    for i = 1, 2 do
      cop_projection = cop_projection + cop_vector[i]*support_vector[i]
    end
    cop_projection = cop_projection/support_norm

    if (cop_projection > 1) then cop_projection = 1 end
    if (cop_projection < 0) then cop_projection = 0 end

    for i = 1, 2 do
      desired_cop[i] = support_sole_position[i]
                     + cop_projection*support_vector[i]
      cop_error[i] = actual_cop[i] - desired_cop[i] 
    end

    -- determine tipping status
    if (cop_projection > 0) and (cop_projection < 1) then
      local cop_error_magnitude = math.sqrt(cop_error[1]^2 + cop_error[2]^2)
      local sole_diagonal = math.sqrt(sole_dimensions[1]^2 + sole_dimensions[2]^2)
      if (cop_error_magnitude > sole_diagonal/2 - tipping_eps) then
        tipping_status = 1
      end
    else
      for i = 1, 2 do
	if (math.abs(cop_error[i]) > sole_dimensions[i]/2 - tipping_eps) then
	  tipping_status = 1
	end
      end
    end
  end

  mcm:set_desired_cop(desired_cop)
  pcm:set_tipping_status(tipping_status)
end

-- Public
--------------------------------------------------------------------------------

function step:set_nominal_initialization(bool)
  nominal_initialization = bool
end

function step:set_support_foot(foot_id) -- l or r
  support_foot = string.lower(foot_id:match('^%a'))
end

function step:set_velocity(velocity_vector)
  copy_array(velocity_vector, velocity)
end

function step:set_foothold(pose)
  -- TODO
end

function step:set_rmp(rmp_object)
  torso_rmp = rmp_object
end

function step:set_rmp_parameters(parameters, dim)
  if (dim) then
    self.parameters.rmp_parameters[dim] = parameters
  else
    self.parameters.rmp_parameters = parameters
  end
end

function step:get_support_foot()
  return support_foot
end

function step:get_velocity()
  return copy_array(velocity)
end

function step:get_foothold()
  -- TODO
end

function step:get_rmp()
  return torso_rmp
end

function step:get_rmp_parameters(dim)
  if (dim) then
    return self.parameters.rmp_parameters[dim]
  else
    return self.parameters.rmp_parameters
  end
end

function step:get_torso_state()
  return torso_state
end

function step:get_torso_reference_state()
  return torso_reference_state
end

function step:get_swing_foot_state()
  return swing_foot_state
end

function step:get_cop_state()
  return cop_state
end

function step:get_configuration()
  -- get current joint positions for both legs
    local torso_frame = Transform.pose(torso_state[1])
    local l_foot_frame, r_foot_frame
    if (support_foot == 'r') then
      r_foot_frame = Transform.pose({0, 0, 0})
      l_foot_frame = Transform.pose(swing_foot_state[1])
    else
      l_foot_frame = Transform.pose({0, 0, 0})
      r_foot_frame = Transform.pose(swing_foot_state[1])
    end
    return Kinematics.inverse_pos_legs(l_foot_frame, r_foot_frame, torso_frame)
end

function step:learn_torso_orbit(xdata, tdata)
  -- learn rmp parameters from periodic training signal
  torso_rmp:learn_trajectory(xdata, tdata)
  self.parameters.rmp_parameters = torso_rmp:get_parameters()
  self.parameters.nominal_rmp_state = torso_rmp:get_state()
end

function step:initialize()
  update_parameters()
  initialize_step_variables()
  initialize_torso_variables()
  initialize_swing_foot_variables()
  update_desired_cop(0)
end

function step:start()
  step:initialize()
  active = true
end

function step:stop()
  active = false
end

function step:is_active()
  return active
end

function step:iterate_nominal_rmp_state()
  -- integrate rmp to improve estimate of intial rmp state
  local current_support_foot = support_foot
  support_foot = 'r'
  self:initialize()
  for i = 1, step_duration/Platform.get_time_step() do
    torso_rmp:integrate()
  end
  local rmp_state = torso_rmp:get_state()
  self:set_parameter('nominal_rmp_state', rmp_state)
  support_foot = current_support_foot
  self:initialize()
end

function step:initialize_simulator_state(duration)
  -- intialize torso position using minimum jerk trajectory
  -- (for simulation initialization)

  -- update initial rmp state
  step:iterate_nominal_rmp_state()
  support_foot = 'l'
  self:initialize()

  -- move torso to initial state
  local torso_mjt = {}
  for i = 1, 3 do
    -- TEMPORARY HACK (WHY DOES SUPPORT FOOT TWIST HAVE OPPOSITE SIGN?)
    local start_position =-support_foot_start_pose[i]
    local goal_position = torso_state[1][i]
    torso_mjt[i] = trajectory.minimum_jerk(start_position, goal_position, duration)
  end

  local t0 = Platform.get_time()
  local t  = t0
  while (t < duration) do
    t = Platform.get_time()
    local torso_position = {} 
    for i = 1, 3 do
      torso_position[i] = torso_mjt[i](t)
    end
    local torso_frame = Transform.pose(torso_position)
    local l_foot_frame, r_foot_frame
    if (support_foot == 'r') then
      r_foot_frame = Transform.pose({0, 0, 0})
      l_foot_frame = Transform.pose(swing_foot_state[1])
    else
      l_foot_frame = Transform.pose({0, 0, 0})
      r_foot_frame = Transform.pose(swing_foot_state[1])
    end
    local q = Kinematics.inverse_pos_legs(l_foot_frame, r_foot_frame, torso_frame)
    dcm:set_joint_position(q, 'legs')
    Platform.update()
  end

  -- initialize simulator physics state
  local torso_twist = torso_state[2]
  Platform.reset_simulator_physics()
  Platform.set_simulator_torso_twist(torso_twist)
end

function step:entry()
  local q0 = dcm:get_joint_position_sensor('legs')
  dcm:set_joint_force(0, 'legs')
  dcm:set_joint_position(q0, 'legs')
  dcm:set_joint_velocity(0, 'legs')
  dcm:set_joint_stiffness(1, 'legs')
  dcm:set_joint_damping(0, 'legs')
  update_parameters()
end

function step:update()
  if active then

    local t = Platform.get_time() - t0
    local dt = Platform.get_time_step()

    update_torso_state(t, dt)
    update_swing_foot_state(t, dt)
    update_desired_cop(t, dt)

    -- update actuators
    local q = self:get_configuration()
    dcm:set_joint_position(q, 'legs')

    if (t >= step_duration) then
      active = false 
    end
  end
end

function step:exit()
end
