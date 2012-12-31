require('numlua')
require('trajectory')

--------------------------------------------------------------------------------
-- dmp : discrete movement primitives
--------------------------------------------------------------------------------

--[[

  author: Mike Hopkins (michael-hopkins@outlook.com)

  Example usage
  -------------
  local dof = 1     -- degrees of freedom
  local dt = 0.001  -- integrator time step
  local nbasis = 20 -- number of basis functions
  local tau = 1     -- movement duration
  local xdata = {}  -- training trajectory
  local tdata = {}  -- training sample times

  -- initialize example trajectory
  for i = 1, math.floor(tau/dt) do
    tdata[i] = i*dt
    xdata[i] = math.sin(math.pi*tdata[i]/2)^2
  end

  -- initialize dmp
  local primitive = dmp.new(dof)
  primitive:set_time_step(dt)
  primitive:learn_trajectory({xdata}, tdata, nbasis)
  primitive:reset()

  -- integrate dmp
  for i = 1, math.floor(tau/dt) do
    primitive:integrate()
    print('t :', i*dt, ' x :', primitive:get_position(1))
  end

  References
  ----------
  "Learning and Generalization of Motor Skills by Learning from 
  Demonstration", ICRA2009

  "Dynamical Movement Primitives: Learning Attractor Models for
  Motor Behaviors", NC2013
  
--]]

dmp = {}
local dmp_nonlinearity = {}
local dmp_canonical_system = {}
local dmp_transform_system = {}

dmp.__index = dmp
dmp_nonlinearity.__index = dmp_nonlinearity
dmp_canonical_system.__index = dmp_canonical_system
dmp_transform_system.__index = dmp_transform_system

-- utilities
--------------------------------------------------------------------------------

local function gaussian(s, center, width)
  return math.exp(-width*(s - center)^2)
end

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

-- dynamic movement primitive
--------------------------------------------------------------------------------

function dmp.new(n_dimensions, k_gain, d_gain, alpha)
  local o = {}
  assert(n_dimensions > 0, "invalid dimensionality")
  o.n_dimensions = n_dimensions -- number of transformation systems
  o.iters        = 1            -- integrator iterations
  o.dt           = nil          -- integrator time step
  o.canonical_system = dmp_canonical_system.new(alpha)
  o.transform_system = {}
  for i = 1,o.n_dimensions do
    o.transform_system[i] = dmp_transform_system.new(k_gain, d_gain)
  end
  return setmetatable(o, dmp)
end

function dmp.set_time_step(o, dt)
  o.dt = dt
  o.canonical_system.dt = dt
  for i = 1,o.n_dimensions do
    o.transform_system[i].dt = dt
  end
end

function dmp.set_integrator_iterations(o, iters)
  o.iters = iters
end

function dmp.init(o, state, g, tau)
  -- initialize dmp default parameters
  -- state           : initial state(s) {x, xd, xdd}
  -- g               : goal position(s)
  -- tau             : duration

  local state = state or {}
  local g = g or {}

  o.canonical_system:init(tau)
  for i = 1,o.n_dimensions do
    o.transform_system[i]:init(state[i], g[i], tau)
  end
end

function dmp.reset(o, state, g, tau)
  -- reset dmp parameters
  -- state           : initial state(s) {x, xd, xdd}
  -- g               : goal position(s)
  -- tau             : duration

  local state = state or {}
  local g = g or {}

  o.canonical_system:reset(tau)
  for i = 1,o.n_dimensions do
    o.transform_system[i]:reset(state[i], g[i], tau)
  end
end

function dmp.set_phase(o, s)
  o.canonical_system.s = s
end

function dmp.set_state(o, state, dim)
  if (dim) then
    o.transform_system[dim]:set_state(state)
  else
    for i = 1,o.n_dimensions do 
      o.transform_system[i]:set_state(state[i])
    end
  end
end

function dmp.set_goal_position(o, g, dim)
  if (dim) then
    o.transform_system[dim]:set_goal_position(g)
  else
    for i = 1,o.n_dimensions do
      o.transform_system[i]:set_goal_position(g[i])
    end
  end
end

function dmp.set_duration(o, tau)
  o.canonical_system.tau = tau
  for i = 1,o.n_dimensions do
    o.transform_system[i].tau = tau
  end
end

function dmp.set_start_position(o, x, dim)
  if (dim) then
    o.transform_system[dim]:set_start_position(x)
  else
    for i = 1,o.n_dimensions do
      o.transform_system[i]:set_start_position(x[i])
    end
  end
end

function dmp.set_parameter_vector(o, theta, dim)
  if (dim) then
    o.transform_system[dim]:set_parameter_vector(theta)
  else
    for i = 1,o.n_dimensions do
      o.transform_system[i]:set_parameter_vector(theta[i])
    end
  end
end

function dmp.get_dimensions(o)
  return o.n_dimensions
end

function dmp.get_phase(o)
  return o.canonical_system.s
end

function dmp.get_position(o, dim)
  if (dim) then
   return o.transform_system[dim].state[1]
  else
    local x = {}
    for i = 1,o.n_dimensions do
      x[i] = o.transform_system[i].state[1]
    end
    return x
  end
end

function dmp.get_velocity(o, dim)
  if (dim) then
   return o.transform_system[dim].state[2]
  else
    local xd = {}
    for i = 1,o.n_dimensions do
      xd[i] = o.transform_system[i].state[2]
    end
    return xd
  end
end

function dmp.get_acceleration(o, dim)
  if (dim) then
   return o.transform_system[dim].state[3]
  else
    local xdd = {} 
    for i = 1,o.n_dimensions do
      xdd[i] = o.transform_system[i].state[3]
    end
    return xdd
  end
end

function dmp.get_state(o, dim)
  if (dim) then
    return copy_array(o.transform_system[dim].state)
  else
    local state = {}
    for i = 1,o.n_dimensions do
      state[i] = copy_array(o.transform_system[i].state)
    end
    return state
  end
end

function dmp.get_goal_position(o, dim)
  if (dim) then
    return o.transform_system[dim].g
  else
    local g = {}
    for i = 1,o.n_dimensions do
      g[i] = o.transform_system[i].g
    end
    return g
  end
end

function dmp.get_duration(o)
  return o.canonical_system.tau
end

function dmp.get_start_position(o, dim)
  if (dim) then
    return o.transform_system[dim].x 
  else
    local x = {}
    for i = 1,o.n_dimensions do
      x[i] = o.transform_system[i].x
    end
    return x
  end
end

function dmp.get_parameter_vector(o, dim)
  if (dim) then
    return o.transform_system[dim]:get_parameter_vector()
  else
    local theta = {}
    for i = 1,o.n_dimensions do
      theta[i] = o.transform_system[i]:get_parameter_vector()
    end
    return theta
  end
end

function dmp.get_basis_vector(o, dim)
  if (dim) then
    local s = o.canonical_system.s
    return o.transform_system[dim]:get_basis_vector(s)
  else
    local psi = {}
    local s = o.canonical_system.s
    for i = 1,o.n_dimensions do
      psi[i] = o.transform_system[i]:get_basis_vector(s)
    end
    return psi
  end
end

function dmp.get_canonical_system(o)
  return o.canonical_system
end

function dmp.get_transform_system(o, dim)
  return o.transform_system[dim]
end

function dmp.learn_minimum_jerk_trajectory(o, nbasis)
  -- learn dmp parameters from minimum jerk trajectory
  -- nbasis          : optional number of basis functions

  local nbasis       = nbasis or 10
  local N            = 5000
  local tdata        = {}
  local xdata        = {}
  local minimum_jerk = {}
  local state        = o:get_state()
  local g            = o:get_goal_position()
  local tau          = o:get_duration()

  -- compute minimum jerk trajectory given x, g and tau
  for i = 1,o.n_dimensions do
    xdata[i] = {}
    minimum_jerk[i] = trajectory.minimum_jerk(state[i], g[i], tau)
  end

  for i = 1,N do
    tdata[i] = tau*(i - 1)/(N - 1)
    for j = 1,o.n_dimensions do
      xdata[j][i] = (minimum_jerk[j])(tdata[i])
    end
  end

  return o:learn_trajectory(xdata, tdata, nbasis)
end

function dmp.learn_trajectory(o, xdata, tdata, nbasis)
  -- learn dmp parameters from discrete time-series trajectory
  -- xdata           : position samples for each DoF  [n_dimensions x N]
  -- tdata           : sample times for each position [1 x N]
  -- nbasis          : optional number of basis functions

  local x            = {}
  local xd           = {}
  local xdd          = {}
  local fdata        = {}
  local sdata        = {}
  local state        = {}
  local g            = {}
  local tau          = tdata[#tdata] - tdata[1]
  assert(#xdata == o.n_dimensions, 'invalid xdata dimensions')

  for i = 1,o.n_dimensions do
    x[i] = xdata[i]
    xd[i] = {}
    xdd[i] = {}
    fdata[i] = {}
    sdata[i] = {}
  end

  -- estimate xd and xdd from training data
  for i = 1,#tdata do
    local ipast = math.max(i - 1, 1)
    local inext = math.min(i + 1, #tdata)
    local dt = (tdata[inext] - tdata[ipast])/2
    for j = 1,o.n_dimensions do
      xd[j][i] = (x[j][inext] - x[j][ipast])/(2*dt)
      xdd[j][i] = (x[j][inext] - 2*x[j][i] + x[j][ipast])/(dt)^2
    end
  end

  for i = 1,o.n_dimensions do
    xd[i][1] = xd[i][2]
    xdd[i][1] = xdd[i][2]
    xd[i][#tdata] = xd[i][#tdata - 1]
    xdd[i][#tdata] = xdd[i][#tdata - 1]
  end

  -- initialize dynamical system
  for i = 1,o.n_dimensions do
    state[i] = {x[i][1], xd[i][1], xdd[i][1]}
    g[i] = x[i][#x[i]]
  end
  o:init(state, g, tau)

  -- calculate nonlinearities
  for i = 1,#tdata do
    local ipast = math.max(i - 1, 1)
    local inext = math.min(i + 1, #tdata)
    local dt = (tdata[inext] - tdata[ipast])/2
    local s = o.canonical_system:integrate(dt)
    for j = 1,o.n_dimensions do
      state[j][1] = x[j][i]
      state[j][2] = xd[j][i]
      state[j][3] = xdd[j][i]
      fdata[j][i] = o.transform_system[j]:estimate_nonlinearity(s, state[j])
    end
    sdata[i] = s
  end

  -- learn nonlinear functions via linear regression
  for i = 1,o.n_dimensions do
    local f = dmp_nonlinearity.new(nbasis, o.canonical_system.alpha)
    f:fit(fdata[i], sdata)
    o.transform_system[i]:set_nonlinearity(f)
  end
  return fdata, sdata
end

function dmp.integrate(o, coupling, dt)
  -- integrate dynamic movement primitive
  -- coupling        : optional additive acceleration vector
  -- dt              : optional time step
  
  local coupling = coupling or {}
  local dt       = dt or o.dt

  for i = 1,o.iters do
    local s = o.canonical_system:integrate(dt/o.iters)
    for j = 1,o.n_dimensions do
      o.transform_system[j]:integrate(s, coupling[j], dt/o.iters)
    end
  end
  return o:get_state()
end

-- nonlinearity
--------------------------------------------------------------------------------

function dmp_nonlinearity.new(nbasis, alpha, theta)
  local o     = {}
  o.nbasis    = nbasis or 20                -- number of basis functions
  o.centers   = {}                          -- basis function centers
  o.widths    = {}                          -- basis function bandwidths
  o.theta     = zeros(o.nbasis)             -- parameter vector
  local alpha = alpha or -math.log(0.01)    -- canonical spring constant

  -- initialize basis functions
  local c0    = (math.exp(alpha/(o.nbasis - 1) - math.exp(-alpha)))
              / (1 - math.exp(-alpha))

  for i = 1,o.nbasis do
    o.centers[i] = math.exp(-alpha*(i - 1)/(o.nbasis - 1))
    o.centers[i] = (o.centers[i] - math.exp(-alpha))/(1 - math.exp(-alpha))
    o.widths[i] = 0.5*(o.centers[i] - (o.centers[i-1] or c0))^(-2)
  end

  -- initialize parameter vector
  if (theta) then
    for i = 1,#theta do
      o.theta[i] = theta[i]
    end
  end

  return setmetatable(o, dmp_nonlinearity)
end

function dmp_nonlinearity.set_basis_centers(o, centers)
  copy_array(centers, o.centers)
end

function dmp_nonlinearity.set_basis_bandwidths(o, widths)
  copy_array(widths, o.widths)
end

function dmp_nonlinearity.set_parameter_vector(o, theta)
  copy_array(theta, o.theta)
end

function dmp_nonlinearity.get_basis_centers(o)
  return copy_array(o.centers)
end

function dmp_nonlinearity.get_basis_bandwidths(o)
  return copy_array(o.widths)
end

function dmp_nonlinearity.get_parameter_vector(o)
  return copy_array(o.theta)
end

function dmp_nonlinearity.get_basis_vector(o, s)
  -- return basis activation vector (for learning parameters)
  -- s               : canonical system state (dmp phase)

  local psi = {}
  local sum = 0

  for i = 1,o.nbasis do
    psi[i] = gaussian(s, o.centers[i], o.widths[i])
    sum = sum + psi[i]
  end
  for i = 1,o.nbasis do
    psi[i] = psi[i]*s/sum
  end

  return psi
end

function dmp_nonlinearity.fit(o, fdata, sdata)
  -- estimate parameter vector via ordinary least squares
  -- fdata           : sampled nonlinearity signal [1 x N]
  -- sdata           : phase values at each sample [1 x N]

  local y = matrix.fromtable(fdata)
  local X = matrix.new(#sdata, o.nbasis)

  for i = 1,#sdata do
    local psi = o:get_basis_vector(sdata[i])
    for j = 1,o.nbasis do
      X[i][j] = psi[j]
    end
  end

  o.theta = (X % y):totable()
end

function dmp_nonlinearity.predict(o, s)
  -- predict f(s) using linear regression
  -- s               : canonical system state (dmp phase)

  local f, sum = 0, 0
  for i = 1,o.nbasis do
    local psi = gaussian(s, o.centers[i], o.widths[i])
    f = f + o.theta[i]*psi
    sum = sum + psi
  end

  return f*s/sum
end

-- canonical system
--------------------------------------------------------------------------------

function dmp_canonical_system.new(alpha)
  local o = {}
  o.alpha = alpha or -math.log(0.01)        -- spring constant 
  o.dt    = nil                             -- integrator time step
  dmp_canonical_system.init(o)
  return setmetatable(o, dmp_canonical_system)
end

function dmp_canonical_system.init(o, tau)
  o.s     = 1                               -- phase
  o.tau   = tau or 1                        -- duration
  o.tau_  = tau or 1                        -- nominal duration
end

function dmp_canonical_system.reset(o, tau)
  o.s     = 1                               -- phase
  o.tau   = tau or o.tau_                   -- duration
end

function dmp_canonical_system.set_time_step(o, dt)
  o.dt = dt
end

function dmp_canonical_system.set_epsilon(o, epsilon)
  -- assign alpha such that s(t) < epsilon after tau seconds
  o.alpha = -math.log(epsilon)
end

function dmp_canonical_system.set_duration(o, tau)
  o.tau  = tau
end

function dmp_canonical_system.set_phase(o, s)
  o.s = s
end

function dmp_canonical_system.get_epsilon(o)
  return math.exp(-o.alpha)
end

function dmp_canonical_system.get_duration(o)
  return o.tau
end

function dmp_canonical_system.get_phase(o)
  return o.s
end

function dmp_canonical_system.integrate(o, dt)
  -- integrate the canonical system using the midpoint method 
  -- dt : optional time step 

  local dt = dt or o.dt
  local ds = 0
  local s  = o.s

  for i = 1,2 do
    ds  = -o.alpha*o.s/o.tau
    o.s = s + ds*dt/(3 - i)
  end

  return o.s
end

-- transformation system
--------------------------------------------------------------------------------

function dmp_transform_system.new(k_gain, d_gain, f)
  local o = {}
  o.k_gain  = k_gain or 250                   -- spring constant
  o.d_gain  = d_gain or 2*math.sqrt(o.k_gain) -- damping coefficient
  o.f       = f or dmp_nonlinearity.new()     -- nonlinear function f(s)
  o.dt      = nil                             -- integrator time step
  dmp_transform_system.init(o)
  return setmetatable(o, dmp_transform_system)
end

function dmp_transform_system.init(o, state, g, tau)
  if type(state) ~= 'table' then
    state = {state}
  end
  state[1] = state[1] or 0
  state[2] = state[2] or 0
  state[3] = state[3] or 0

  o.state   = copy_array(state)               -- state {x, xd, xdd}
  o.state_  = copy_array(state)               -- nominal state
  o.g       = g or 0                          -- goal position
  o.g_      = g or 0                          -- nominal goal position
  o.tau     = tau or 1                        -- duration
  o.tau_    = tau or 1                        -- nominal duration
  o.x0      = o.state[1]                      -- start position
end

function dmp_transform_system.reset(o, state, g, tau)
  o:set_state(state or o.state_)
  o:set_goal_position(g or o.g_)
  o:set_duration(tau or o.tau_)
  o:set_start_position(o.state[1])
end

function dmp_transform_system.set_time_step(o, dt)
  o.dt = dt
end

function dmp_transform_system.set_state(o, state)
  if type(state) ~= 'table' then
    state = {state}
  end
  o.state[1] = state[1] or 0
  o.state[2] = state[2] or 0
  o.state[3] = state[3] or 0
end

function dmp_transform_system.set_goal_position(o, g)
  o.g = g
end

function dmp_transform_system.set_duration(o, tau)
  o.tau = tau
end

function dmp_transform_system.set_start_position(o, x0)
  o.x0 = x0
end

function dmp_transform_system.set_parameter_vector(o, theta)
  o.f:set_parameter_vector(theta)
end

function dmp_transform_system.set_nonlinearity(o, f)
  o.f = f
end

function dmp_transform_system.get_state(o)
  return copy_array(state)
end

function dmp_transform_system.get_goal_position(o)
  return o.g
end

function dmp_transform_system.get_duration(o)
  return o.tau
end

function dmp_transform_system.get_start_position(o)
  return o.x0
end

function dmp_transform_system.get_parameter_vector(o)
  return o.f:get_parameter_vector()
end

function dmp_transform_system.get_basis_vector(o, s)
  return o.f:get_basis_vector(s)
end

function dmp_transform_system.get_nonlinearity(o)
  return o.f
end

function dmp_transform_system.estimate_nonlinearity(o, s, state)
  -- estimate the nonlinearity f(s) given the desired phase / state
  -- s             : canonical system state (dmp phase)
  -- state         : desired system state {x, xd, xdd}

  local fs
  local x, xd, xdd = unpack(state)

  fs = (xdd*o.tau^2 + o.d_gain*(xd*o.tau))/o.k_gain
     - (o.g - x) + (o.g - o.x0)*s

  return fs
end

function dmp_transform_system.integrate(o, s, coupling, dt)
  -- integrate the transformation system using the velocity verlet method
  -- s             : canonical system state (dmp phase)
  -- coupling      : optional additive acceleration term
  -- dt            : optional time step

  local coupling   = coupling or 0
  local dt         = dt or o.dt
  local fs         = o.f:predict(s)
  local x, xd, xdd = unpack(o.state)

  xd  = xd + 1/2*xdd*dt
  x   = x + xd*dt
  xdd = o.k_gain*(o.g - x) - o.d_gain*(xd*o.tau)
      - o.k_gain*(o.g - o.x0)*s + o.k_gain*fs
  xdd = xdd/o.tau^2 + coupling
  xd  = xd + 1/2*xdd*dt

  o.state[1] = x
  o.state[2] = xd
  o.state[3] = xdd
  return {x, xd, xdd} 
end

return dmp
