require('numlua')

--------------------------------------------------------------------------------
-- rmp : rhythmic movement primitives
--------------------------------------------------------------------------------

--[[

  author: Mike Hopkins (michael-hopkins@outlook.com)

  Example usage
  -------------
  local n_dimensions = 1 -- degrees of freedom
  local n_basis = 20     -- number of basis functions
  local dt = 0.001       -- integrator time step
  local tau = 1          -- movement period
  local xdata = {}       -- training trajectory
  local tdata = {}       -- training sample times

  -- initialize example trajectory
  for i = 1, math.floor(tau/dt) do
    tdata[i] = i*dt
    xdata[i] = math.cos(2*math.pi*tdata[i])
  end

  -- initialize rmp
  local primitive = rmp.new(n_dimensions, n_basis)
  primitive:set_time_step(dt)
  primitive:set_setpoint({0}) -- define setpoint prior to training
  primitive:learn_trajectory({xdata}, tdata)
  primitive:reset()

  -- integrate rmp
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

rmp = {}
rmp.nonlinearity = {}
rmp.canonical_system = {}
rmp.transform_system = {}

rmp.__index = rmp
rmp.nonlinearity.__index = rmp.nonlinearity
rmp.canonical_system.__index = rmp.canonical_system
rmp.transform_system.__index = rmp.transform_system

rmp.__mtstring = 'rmp'
rmp.nonlinearity.__mtstring = 'rmp.nonlinearity'
rmp.canonical_system.__mtstring = 'rmp.canonical_system'
rmp.transform_system.__mtstring = 'rmp.transform_system'

-- utilities
--------------------------------------------------------------------------------

local function von_mises(s, center, width)
  local cs = math.cos(s - center)
  return math.exp(width*(cs - 1))
end

local function antiperiodic_von_mises(s, center, width)
  local cs = math.cos((s - center)/2)
  return cs*math.exp(2*width*(cs^2 - 1))
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

-- rhythmic movement primitive
--------------------------------------------------------------------------------

function rmp.new(n_dimensions, n_basis, basis_type, k_gain, d_gain, alpha)
  local o = {}
  o.n_dimensions = n_dimensions -- number of transformation systems
  o.iters        = 1            -- integrator iterations
  o.dt           = nil          -- integrator time step
  assert(o.n_dimensions > 0, "invalid dimensionality")

  -- initialize number of basis functions per dimension
  if (type(n_basis) ~= 'table') then
    local value = n_basis 
    n_basis = {}
    for i = 1,o.n_dimensions do
      n_basis[i] = value 
    end
  end

  -- intialize basis type for each dimension 
  if (type(basis_type) ~= 'table') then
    local value = basis_type 
    basis_type = {}
    for i = 1,o.n_dimensions do
      basis_type[i] = value 
    end
  end

  -- intialize canonical and transformation systems
  o.canonical_system = rmp.canonical_system.new(alpha)
  o.transform_system = {}
  for i = 1,o.n_dimensions do
    o.transform_system[i] =
      rmp.transform_system.new(n_basis[i], basis_type[i], k_gain, d_gain)
  end
  return setmetatable(o, rmp)
end

function rmp.set_time_step(o, dt)
  o.dt = dt
  o.canonical_system.dt = dt
  for i = 1,o.n_dimensions do
    o.transform_system[i].dt = dt
  end
end

function rmp.set_integrator_iterations(o, iters)
  o.iters = iters
end

function rmp.initialize(o, state, g, a, tau)
  -- initialize rmp default parameters
  -- state           : initial state(s) {x, xd, xdd}
  -- g               : setpoint(s)
  -- a               : amplitude(s)
  -- tau             : period

  local state = state or {}
  local g = g or {}
  local a = a or {}

  o.canonical_system:initialize(tau)
  for i = 1,o.n_dimensions do
    o.transform_system[i]:initialize(state[i], g[i], a[i], tau)
  end
end

function rmp.reset(o, state, g, a, tau)
  -- reset rmp parameters
  -- state           : initial state(s) {x, xd, xdd}
  -- g               : setpoint(s)
  -- a               : amplitude(s)
  -- tau             : period

  local state = state or {}
  local g = g or {}
  local a = a or {}

  o.canonical_system:reset(tau)
  for i = 1,o.n_dimensions do
    o.transform_system[i]:reset(state[i], g[i], a[i], tau)
  end
end

function rmp.set_phase(o, s)
  o.canonical_system.s = s
end

function rmp.set_state(o, state, dim)
  if (dim) then
    o.transform_system[dim]:set_state(state)
  else
    for i = 1,o.n_dimensions do 
      o.transform_system[i]:set_state(state[i])
    end
  end
end

function rmp.set_setpoint(o, g, dim)
  if (dim) then
    o.transform_system[dim]:set_setpoint(g)
  else
    for i = 1,o.n_dimensions do
      o.transform_system[i]:set_setpoint(g[i])
    end
  end
end

function rmp.set_amplitude(o, a, dim)
  if (dim) then
    o.transform_system[dim]:set_amplitude(a)
  else
    for i = 1,o.n_dimensions do
      o.transform_system[i]:set_amplitude(a[i])
    end
  end
end

function rmp.set_period(o, tau)
  o.canonical_system.tau = tau
  for i = 1,o.n_dimensions do
    o.transform_system[i].tau = tau
  end
end

function rmp.set_parameters(o, theta, dim)
  if (dim) then
    o.transform_system[dim]:set_parameters(theta)
  else
    for i = 1,o.n_dimensions do
      o.transform_system[i]:set_parameters(theta[i])
    end
  end
end

function rmp.get_dimensions(o)
  return o.n_dimensions
end

function rmp.get_phase(o)
  return o.canonical_system.s
end

function rmp.get_position(o, dim)
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

function rmp.get_velocity(o, dim)
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

function rmp.get_acceleration(o, dim)
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

function rmp.get_state(o, dim)
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

function rmp.get_setpoint(o, dim)
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

function rmp.get_amplitude(o, dim)
  if (dim) then
    return o.transform_system[dim].a
  else
    local a = {}
    for i = 1,o.n_dimensions do
      a[i] = o.transform_system[i].a
    end
    return a
  end
end

function rmp.get_period(o)
  return o.canonical_system.tau
end

function rmp.get_parameters(o, dim)
  if (dim) then
    return o.transform_system[dim]:get_parameters()
  else
    local theta = {}
    for i = 1,o.n_dimensions do
      theta[i] = o.transform_system[i]:get_parameters()
    end
    return theta
  end
end

function rmp.get_basis_vector(o, dim)
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

function rmp.get_canonical_system(o)
  return o.canonical_system
end

function rmp.get_transform_system(o, dim)
  return o.transform_system[dim]
end

function rmp.learn_trajectory(o, xdata, tdata)
  -- learn rmp parameters from periodic or antiperiodic time-series trajectory
  -- xdata           : position samples for each DoF  [n_dimensions x N]
  -- tdata           : sample times for each position [1 x N]

  local x            = {}
  local xd           = {}
  local xdd          = {}
  local fdata        = {}
  local sdata        = {}
  local state        = {}
  local g            = o:get_setpoint()
  local a            = ones(o.n_dimensions)
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
  end
  o:initialize(state, g, a, tau)

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
    local f = o.transform_system[i]:get_nonlinearity()
    f:fit(fdata[i], sdata)
  end

  o:reset()
  return fdata, sdata
end

function rmp.integrate(o, coupling, dt)
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

function rmp.nonlinearity.new(n_basis, basis_type, theta)
  local o     = {}
  o.n_basis   = n_basis or 20               -- number of basis functions
  o.basis     = nil                         -- basis function
  o.centers   = {}                          -- basis function centers
  o.widths    = {}                          -- basis function widths
  o.theta     = zeros(o.n_basis)            -- parameter vector

  -- initialize basis functions
  if (basis_type == 'antiperiodic') then
    o.basis = antiperiodic_von_mises
  else
    o.basis = von_mises
  end

  for i = 1,o.n_basis do
    o.centers[i] = 2*math.pi*(i - 1)/o.n_basis
    o.widths[i] = o.n_basis
  end

  -- initialize parameter vector
  if (theta) then
    for i = 1,#theta do
      o.theta[i] = theta[i]
    end
  end

  return setmetatable(o, rmp.nonlinearity)
end

function rmp.nonlinearity.set_basis_centers(o, centers)
  copy_array(centers, o.centers)
end

function rmp.nonlinearity.set_basis_bandwidths(o, widths)
  copy_array(widths, o.widths)
end

function rmp.nonlinearity.set_parameters(o, theta)
  copy_array(theta, o.theta)
end

function rmp.nonlinearity.get_basis_centers(o)
  return copy_array(o.centers)
end

function rmp.nonlinearity.get_basis_bandwidths(o)
  return copy_array(o.widths)
end

function rmp.nonlinearity.get_parameters(o)
  return copy_array(o.theta)
end

function rmp.nonlinearity.get_basis_vector(o, s)
  -- return basis activation vector (for learning parameters)
  -- s               : canonical system state (rmp phase)

  local psi = {}
  local sum = 0

  for i = 1,o.n_basis do
    psi[i] = o.basis(s, o.centers[i], o.widths[i])
    sum = sum + math.abs(psi[i])
  end
  for i = 1,o.n_basis do
    psi[i] = psi[i]/sum
  end

  return psi
end

function rmp.nonlinearity.fit(o, fdata, sdata)
  -- calculate basis weights via ordinary least squares
  -- fdata           : sampled nonlinearity signal [1 x N]
  -- sdata           : phase values at each sample [1 x N]

  local y = matrix.fromtable(fdata)
  local X = matrix.new(#sdata, o.n_basis)

  for i = 1,#sdata do
    local psi = o:get_basis_vector(sdata[i])
    for j = 1,o.n_basis do
      X[i][j] = psi[j]
    end
  end

  o.theta = (X % y):totable()
end

function rmp.nonlinearity.predict(o, s)
  -- predict f(s) using linear regression
  -- s               : canonical system state (rmp phase)

  local f, sum = 0, 0
  for i = 1,o.n_basis do
    local psi = o.basis(s, o.centers[i], o.widths[i])
    f = f + o.theta[i]*psi
    sum = sum + math.abs(psi)
  end

  return f/sum
end

-- canonical system
--------------------------------------------------------------------------------

function rmp.canonical_system.new()
  local o = {}
  o.dt    = nil                             -- integrator time step
  rmp.canonical_system.initialize(o)
  return setmetatable(o, rmp.canonical_system)
end

function rmp.canonical_system.initialize(o, tau)
  o.s     = 0                               -- phase
  o.tau   = tau or 1                        -- period
  o.tau_  = tau or 1                        -- nominal period
end

function rmp.canonical_system.reset(o, tau)
  o.s     = 0                               -- phase
  o.tau   = tau or o.tau_                   -- period
end

function rmp.canonical_system.set_time_step(o, dt)
  o.dt = dt
end

function rmp.canonical_system.set_period(o, tau)
  o.tau  = tau
end

function rmp.canonical_system.set_phase(o, s)
  o.s = s
end

function rmp.canonical_system.get_period(o)
  return o.tau
end

function rmp.canonical_system.get_phase(o)
  return o.s
end

function rmp.canonical_system.integrate(o, dt)
  -- integrate the canonical system
  -- dt : optional time step 

  local dt = dt or o.dt
  o.s = o.s + 2*math.pi*dt/o.tau
  return o.s
end

-- transformation system
--------------------------------------------------------------------------------

function rmp.transform_system.new(n_basis, basis_type, k_gain, d_gain)
  local o = {}
  o.k_gain  = k_gain or 500                   -- spring constant
  o.d_gain  = d_gain or 2*math.sqrt(o.k_gain) -- damping coefficient
  o.dt      = nil                             -- integrator time step
  o.f       = rmp.nonlinearity.new(n_basis, basis_type)
  rmp.transform_system.initialize(o)
  return setmetatable(o, rmp.transform_system)
end

function rmp.transform_system.initialize(o, state, g, a, tau)
  if type(state) ~= 'table' then
    state = {state}
  end
  state[1] = state[1] or 0
  state[2] = state[2] or 0
  state[3] = state[3] or 0

  o.state   = copy_array(state)               -- state {x, xd, xdd}
  o.state_  = copy_array(state)               -- nominal state
  o.g       = g or 0                          -- setpoint
  o.g_      = g or 0                          -- nominal setpoint
  o.a       = a or 1                          -- amplitude
  o.a_      = a or 1                          -- nominal amplitude
  o.tau     = tau or 1                        -- period
  o.tau_    = tau or 1                        -- nominal period
end

function rmp.transform_system.reset(o, state, g, a, tau)
  o:set_state(state or o.state_)
  o:set_setpoint(g or o.g_)
  o:set_amplitude(a or o.a_)
  o:set_period(tau or o.tau_)
end

function rmp.transform_system.set_time_step(o, dt)
  o.dt = dt
end

function rmp.transform_system.set_state(o, state)
  if type(state) ~= 'table' then
    state = {state}
  end
  o.state[1] = state[1] or 0
  o.state[2] = state[2] or 0
  o.state[3] = state[3] or 0
end

function rmp.transform_system.set_setpoint(o, g)
  o.g = g
end

function rmp.transform_system.set_amplitude(o, a)
  o.a = a
end

function rmp.transform_system.set_period(o, tau)
  o.tau = tau
end

function rmp.transform_system.set_parameters(o, theta)
  o.f:set_parameters(theta)
end

function rmp.transform_system.set_nonlinearity(o, f)
  o.f = f
end

function rmp.transform_system.get_state(o)
  return copy_array(state)
end

function rmp.transform_system.get_setpoint(o)
  return o.g
end

function rmp.transform_system.get_amplitude(o)
  return o.a
end

function rmp.transform_system.get_period(o)
  return o.tau
end

function rmp.transform_system.get_parameters(o)
  return o.f:get_parameters()
end

function rmp.transform_system.get_basis_vector(o, s)
  return o.f:get_basis_vector(s)
end

function rmp.transform_system.get_nonlinearity(o)
  return o.f
end

function rmp.transform_system.estimate_nonlinearity(o, s, state)
  -- estimate the nonlinearity f(s) given the desired phase / state
  -- s             : canonical system state (rmp phase)
  -- state         : desired system state {x, xd, xdd}

  local fs
  local x, xd, xdd = unpack(state)

  fs = (xdd*o.tau^2 + o.d_gain*(xd*o.tau))/(o.a*o.k_gain)
     - (o.g - x)/o.a

  return fs
end

function rmp.transform_system.integrate(o, s, coupling, dt)
  -- integrate the transformation system using the velocity verlet method
  -- s             : canonical system state (rmp phase)
  -- coupling      : optional additive acceleration term
  -- dt            : optional time step

  local coupling   = coupling or 0
  local dt         = dt or o.dt
  local fs         = o.f:predict(s)
  local x, xd, xdd = unpack(o.state)

  xd  = xd + 1/2*xdd*dt
  x   = x + xd*dt
  xdd = o.k_gain*(o.g - x) - o.d_gain*(xd*o.tau)
      + o.a*o.k_gain*fs
  xdd = xdd/o.tau^2 + coupling
  xd  = xd + 1/2*xdd*dt

  o.state[1] = x
  o.state[2] = xd
  o.state[3] = xdd
  return {x, xd, xdd} 
end

return rmp
