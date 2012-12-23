require('numlua')

---------------------------------------------------------------------------
-- rmp : rhythmic movement primitives
---------------------------------------------------------------------------

--[[

  author: Mike Hopkins (michael-hopkins@outlook.com)

  Example usage
  -------------
  local dof = 1     -- degrees of freedom
  local dt = 0.001  -- integrator time step
  local nbasis = 20 -- number of basis functions
  local tau = 1     -- movement period
  local xdata = {}  -- training trajectory
  local tdata = {}  -- training sample times

  -- initialize example trajectory
  for i = 1, math.floor(tau/dt) do
    tdata[i] = i*dt
    xdata[i] = math.cos(2*math.pi*tdata[i])
  end

  -- initialize rmp
  local primitive = rmp.new(dof)
  primitive:set_time_step(dt)
  primitive:set_setpoint({0})
  primitive:learn_trajectory({xdata}, tdata, nbasis, 'periodic')
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
local rmp_nonlinearity = {}
local rmp_canonical_system = {}
local rmp_transform_system = {}

rmp.__index = rmp
rmp_nonlinearity.__index = rmp_nonlinearity
rmp_canonical_system.__index = rmp_canonical_system
rmp_transform_system.__index = rmp_transform_system

-- utilities
---------------------------------------------------------------------------

local function von_mises(s, center, width)
  local cs = math.cos(s - center)
  return math.exp(width*(cs - 1))
end

local function antiperiodic_von_mises(s, center, width)
  local cs = math.cos(s - center)
  return cs*math.exp(width*(cs*cs - 1))
end

local function zeros(n)
  local t = {}
  for i = 1,n do
    t[i] = 0
  end
  return t
end

local function copy_array(t)
  if not t then return nil end
  local c = {}
  for i = 1,#t do
    c[i] = t[i]
  end
  return c
end

-- rhythmic movement primitive
---------------------------------------------------------------------------

function rmp.new(ndims, k_gain, d_gain, alpha)
  local o = {}
  assert(ndims > 0, "invalid dimensionality")
  o.ndims = ndims -- number of transformation systems
  o.iters = 1     -- integrator iterations
  o.dt    = nil   -- integrator time step
  o.canonical_system = rmp_canonical_system.new(alpha)
  o.transform_system = {}
  for i = 1,o.ndims do
    o.transform_system[i] = rmp_transform_system.new(k_gain, d_gain)
  end
  return setmetatable(o, rmp)
end

function rmp.set_time_step(o, dt)
  o.dt = dt
  o.canonical_system.dt = dt
  for i = 1,o.ndims do
    o.transform_system[i].dt = dt
  end
end

function rmp.set_integrator_iterations(o, iters)
  o.iters = iters
end

function rmp.init(o, state, g, a, tau)
  -- initialize rmp default parameters
  -- state           : initial state(s) {x, xd, xdd}
  -- g               : setpoint(s)
  -- a               : amplitude(s)
  -- tau             : period

  local state = state or {}
  local g = g or {}
  local a = a or {}

  o.canonical_system:init(tau)
  for i = 1,o.ndims do
    o.transform_system[i]:init(state[i], g[i], a[i], tau)
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
  for i = 1,o.ndims do
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
    for i = 1,o.ndims do 
      o.transform_system[i]:set_state(state[i])
    end
  end
end

function rmp.set_setpoint(o, g, dim)
  if (dim) then
    o.transform_system[dim]:set_setpoint(g)
  else
    for i = 1,o.ndims do
      o.transform_system[i]:set_setpoint(g[i])
    end
  end
end

function rmp.set_amplitude(o, a, dim)
  if (dim) then
    o.transform_system[dim]:set_amplitude(a)
  else
    for i = 1,o.ndims do
      o.transform_system[i]:set_amplitude(a[i])
    end
  end
end

function rmp.set_period(o, tau)
  o.canonical_system.tau = tau
  for i = 1,o.ndims do
    o.transform_system[i].tau = tau
  end
end

function rmp.get_phase(o)
  return o.canonical_system.s
end

function rmp.get_position(o, dim)
  if (dim) then
   return o.transform_system[dim].state[1]
  else
    local x = {}
    for i = 1,o.ndims do
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
    for i = 1,o.ndims do
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
    for i = 1,o.ndims do
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
    for i = 1,o.ndims do
      state[i] = copy_array(o.transform_system[i].state)
    end
    return state
  end
end

function rmp.get_setpoint(o, dim)
  if (dim) then
    return o.transform_system[i].g
  else
    local g = {}
    for i = 1,o.ndims do
      g[i] = o.transform_system[i].g
    end
    return g
  end
end

function rmp.get_amplitude(o, dim)
  if (dim) then
    return o.transform_system[i].a
  else
    local a = {}
    for i = 1,o.ndims do
      a[i] = o.transform_system[i].a
    end
    return a
  end
end

function rmp.get_period(o)
  return o.canonical_system.tau
end

function rmp.get_canonical_system(o)
  return o.canonical_system
end

function rmp.get_transform_system(o, dim)
  return o.transform_system[dim]
end

function rmp.learn_trajectory(o, xdata, tdata, nbasis, basis_type)
  -- learn rmp parameters from periodic or antiperiodic time-series trajectory
  -- xdata           : position samples for each DoF  [ndims x N]
  -- tdata           : sample times for each position [1 x N]
  -- nbasis          : optional number of basis functions
  -- basis_type      : 'periodic' or 'antiperiodic'

  local x            = {}
  local xd           = {}
  local xdd          = {}
  local fdata        = {}
  local sdata        = {}
  local state        = {}
  local g            = o:get_setpoint()
  local a            = o:get_amplitude()
  local tau          = tdata[#tdata] - tdata[1]
  assert(#xdata == o.ndims, 'invalid xdata dimensions')

  for i = 1,o.ndims do
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
    for j = 1,o.ndims do
      xd[j][i] = (x[j][inext] - x[j][ipast])/(2*dt)
      xdd[j][i] = (x[j][inext] - 2*x[j][i] + x[j][ipast])/(dt)^2
    end
  end

  for i = 1,o.ndims do
    xd[i][1] = xd[i][2]
    xdd[i][1] = xdd[i][2]
    xd[i][#tdata] = xd[i][#tdata - 1]
    xdd[i][#tdata] = xdd[i][#tdata - 1]
  end

  -- initialize dynamical system
  for i = 1,o.ndims do
    state[i] = {x[i][1], xd[i][1], xdd[i][1]}
  end
  o:init(state, g, a, tau)

  -- calculate nonlinearities
  for i = 1,#tdata do
    local ipast = math.max(i - 1, 1)
    local inext = math.min(i + 1, #tdata)
    local dt = (tdata[inext] - tdata[ipast])/2
    local s = o.canonical_system:integrate(dt)
    for j = 1,o.ndims do
      state[j][1] = x[j][i]
      state[j][2] = xd[j][i]
      state[j][3] = xdd[j][i]
      fdata[j][i] = o.transform_system[j]:estimate_nonlinearity(s, state[j])
    end
    sdata[i] = s
  end

  -- learn nonlinear functions via linear regression
  for i = 1,o.ndims do
    local f = rmp_nonlinearity.new(nbasis, basis_type)
    f:fit(fdata[i], sdata)
    o.transform_system[i]:set_nonlinearity(f)
  end
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
    for j = 1,o.ndims do
      o.transform_system[j]:integrate(s, coupling[j], dt/o.iters)
    end
  end
  return o:get_state()
end

-- nonlinearity
---------------------------------------------------------------------------

function rmp_nonlinearity.new(nbasis, basis_type, theta)
  local o     = {}
  o.nbasis    = nbasis or 10                -- number of basis functions
  o.center    = {}                          -- basis function centers
  o.width     = {}                          -- basis function widths
  o.theta     = zeros(o.nbasis)             -- basis function weigths
  o.basis     = nil                         -- basis function

  -- initialize basis function
  if (basis_type == 'antiperiodic') then
    o.basis = antiperiodic_von_mises
  else
    o.basis = von_mises
  end

  -- initialize basis parameters
  for i = 1,o.nbasis do
    if (basis_type == 'antiperiodic') then
      o.center[i] = 1*math.pi*(i - 1)/o.nbasis
    else
      o.center[i] = 2*math.pi*(i - 1)/o.nbasis 
    end
    o.width[i] = o.nbasis
  end

  -- initialize basis weights
  if (theta) then
    for i = 1,#theta do
      o.theta[i] = theta[i]
    end
  end

  return setmetatable(o, rmp_nonlinearity)
end

function rmp_nonlinearity.get_basis_centers(o)
  return copy_array(o.center)
end

function rmp_nonlinearity.get_basis_widths(o)
  return copy_array(o.width)
end

function rmp_nonlinearity.get_basis_weights(o)
  return copy_array(o.theta)
end

function rmp_nonlinearity.set_basis_centers(o, centers)
  o.center = copy_array(centers)
end

function rmp_nonlinearity.set_basis_widths(o, widths)
  o.width = copy_array(widths)
end

function rmp_nonlinearity.set_basis_weights(o, thetas)
  o.theta = copy_array(weights)
end

function rmp_nonlinearity.fit(o, fdata, sdata)
  -- calculate basis weights via ordinary least squares
  -- fdata           : sampled nonlinearity signal [1 x N]
  -- sdata           : phase values at each sample [1 x N]

  local y = matrix.fromtable(fdata)
  local X = matrix.new(#sdata, o.nbasis)

  for i = 1,#sdata do
    local psi = matrix.new(o.nbasis)
    for j = 1,o.nbasis do
      psi[j] = o.basis(sdata[i], o.center[j], o.width[j])
    end
    local sum = matrix.sum(psi)
    for j = 1,o.nbasis do
      X[i][j] = psi[j]/sum
    end
  end

  o.theta = (X % y):totable()
end

function rmp_nonlinearity.predict(o, s)
  -- predict y = f(s) using linear regression
  -- s               : canonical system state (rmp phase)

  local f, sum = 0, 0
  for i = 1,o.nbasis do
    local psi = o.basis(s, o.center[i], o.width[i])
    f = f + o.theta[i]*psi
    sum = sum + psi
  end

  return f/sum
end

-- canonical system
---------------------------------------------------------------------------

function rmp_canonical_system.new()
  local o = {}
  o.dt    = nil                             -- integrator time step
  rmp_canonical_system.init(o)
  return setmetatable(o, rmp_canonical_system)
end

function rmp_canonical_system.init(o, tau)
  o.s     = 0                               -- phase
  o.tau   = tau or 1                        -- period
  o.tau_  = tau or 1                        -- nominal period
end

function rmp_canonical_system.reset(o, tau)
  o.s     = 0                               -- phase
  o.tau   = tau or o.tau_                   -- period
end

function rmp_canonical_system.set_time_step(o, dt)
  o.dt = dt
end

function rmp_canonical_system.set_period(o, tau)
  o.tau  = tau
end

function rmp_canonical_system.set_phase(o, s)
  o.s = s
end

function rmp_canonical_system.get_period(o)
  return o.tau
end

function rmp_canonical_system.get_phase(o)
  return o.s
end

function rmp_canonical_system.integrate(o, dt)
  -- integrate the canonical system
  -- dt : optional time step 

  local dt = dt or o.dt
  o.s = o.s + dt/o.tau
  return o.s
end

-- transformation system
---------------------------------------------------------------------------

function rmp_transform_system.new(k_gain, d_gain, f)
  local o = {}
  o.k_gain  = k_gain or 250                   -- spring constant
  o.d_gain  = d_gain or 2*math.sqrt(o.k_gain) -- damping coefficient
  o.f       = f or rmp_nonlinearity.new()     -- nonlinear function f(s)
  o.dt      = nil                             -- integrator time step
  rmp_transform_system.init(o)
  return setmetatable(o, rmp_transform_system)
end

function rmp_transform_system.init(o, state, g, a, tau)
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

function rmp_transform_system.reset(o, state, g, a, tau)
  o:set_state(state or o.state_)
  o:set_setpoint(g or o.g_)
  o:set_amplitude(a or o.a_)
  o:set_period(tau or o.tau_)
end

function rmp_transform_system.set_time_step(o, dt)
  o.dt = dt
end

function rmp_transform_system.set_state(o, state)
  if type(state) ~= 'table' then
    state = {state}
  end
  o.state[1] = state[1] or 0
  o.state[2] = state[2] or 0
  o.state[3] = state[3] or 0
end

function rmp_transform_system.set_setpoint(o, g)
  o.g = g
end

function rmp_transform_system.set_amplitude(o, a)
  o.a = a
end

function rmp_transform_system.set_period(o, tau)
  o.tau = tau
end

function rmp_transform_system.set_nonlinearity(o, f)
  o.f = f
end

function rmp_transform_system.get_state(o)
  return copy_array(state)
end

function rmp_transform_system.get_setpoint(o)
  return o.g
end

function rmp_transform_system.get_amplitude(o)
  return o.a
end

function rmp_transform_system.get_period(o)
  return o.tau
end

function rmp_transform_system.get_nonlinearity(o)
  return o.f
end

function rmp_transform_system.estimate_nonlinearity(o, s, state)
  -- estimate the nonlinearity f(s) given the desired phase / state
  -- s             : canonical system state (rmp phase)
  -- state         : desired system state {x, xd, xdd}

  local fs
  local x, xd, xdd = unpack(state)

  fs = (xdd*o.tau^2 + o.d_gain*(xd*o.tau))/(o.a*o.k_gain)
     - (o.g - x)/o.a

  return fs
end

function rmp_transform_system.integrate(o, s, coupling, dt)
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
