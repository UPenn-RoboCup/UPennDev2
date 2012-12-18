require('numlua')
require('trajectory')

----------------------------------------------------------------------
-- dmp : dynamic movement primitives
----------------------------------------------------------------------

--[[

  author: Mike Hopkins (michael-hopkins@outlook.com)

  Example usage
  -------------
  local dof = 6     -- degrees of freedom
  local dt = 0.001  -- integrator time step
  local nbasis = 20 -- number of radial basis functions

  -- initialize dmp
  local primitive = dmp.new(dof)
  primitive:set_time_step(dt)
  primitive:learn_trajectory(xdata, tdata, nbasis)
  primitive:reset()

  -- integrate dmp
  local tau = primitive:get_duration()
  for i = 1, math.floor(tau/dt) do
    local dmp_state = primitive:integrate()
  end

  References
  ----------
  "Learning and Generalization of Motor Skills by Learning from 
  Demonstration", ICRA2009
  
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
----------------------------------------------------------------------

local function gaussian(x, center, width)
  return math.exp(-width*(x - center)^2)
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

-- dynamic movement primitive
----------------------------------------------------------------------

function dmp.new(ndims, canononical_system, transform_system)
  local o = {}
  assert(ndims > 0, "invalid dimensionality")
  o.ndims = ndims -- number of transformation systems
  o.iters = 1     -- integrator iterations
  o.dt    = nil   -- integrator time step
  o.canonical_system = canonical_system or dmp_canonical_system.new()
  o.canonical_system:reset()
  o.transform_system = {} 
  for i = 1,o.ndims do
    if (transform_system and transform_system[i]) then
      o.transform_system[i] = transform_system[i]
    else
      o.transform_system[i] = dmp_transform_system.new()
    end
    o.transform_system[i]:reset()
    o.transform_system[i]:set_duration(o.canonical_system:get_duration())
  end
  return setmetatable(o, dmp)
end

function dmp.set_time_step(o, dt)
  o.dt = dt
  o.canonical_system.dt = dt
  for i = 1,o.ndims do
    o.transform_system[i].dt = dt
  end
end

function dmp.set_integrator_iterations(o, iters)
  o.iters = iters
end

function dmp.reset(o, x, g, tau)
  local x = x or {}
  local g = g or {}
  o.canonical_system:reset(tau)
  for i = 1,o.ndims do
    o.transform_system[i]:reset(x[i], g[i], tau)
  end
end

function dmp.set_phase(o, s)
  o.canonical_system.s = s
end

function dmp.set_start_position(o, x)
  for i = 1,o.ndims do
    o.transform_system[i].x = x[i]
  end
end

function dmp.set_goal_position(o, g)
  for i = 1,o.ndims do
    o.transform_system[i].g = g[i]
  end
end

function dmp.set_duration(o, tau)
  o.canonical_system.tau = tau
  for i = 1,o.ndims do
    o.transform_system[i].tau = tau
  end
end

function dmp.set_state(o, state, dim)
  if (dim) then
    o.transform_system[dim].state = copy_array(state)
  else
    for i = 1,o.ndims do 
      if (state[i]) then
        o.transform_system[i].state = copy_array(state[i])
      end
    end
  end
end

function dmp.get_phase(o)
  return o.canonical_system.s
end

function dmp.get_position(o, dim)
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

function dmp.get_velocity(o, dim)
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

function dmp.get_acceleration(o, dim)
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

function dmp.get_start_position(o)
  local x = {}
  for i = 1,o.ndims do
    x[i] = o.transform_system[i].x
  end
  return x
end

function dmp.get_goal_position(o)
  local g = {}
  for i = 1,o.ndims do
    g[i] = o.transform_system[i].g
  end
  return g
end

function dmp.get_duration(o)
  return o.canonical_system.tau
end

function dmp.get_state(o, dim)
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

function dmp.get_canonical_system(o)
  return o.canonical_system
end

function dmp.get_transform_system(o, dim)
  return o.transform_system[dim]
end

function dmp.init(o, x, g, tau, k_gain, d_gain)
  -- initialize dmp parameters to unforced spring and damper system 
  -- x               : start position
  -- g               : goal position
  -- tau             : duration
  -- k_gain          : optional spring constant
  -- d_gain          : optional damping coefficient

  o.canonical_system:init(tau)
  for i = 1,o.ndims do
    o.transform_system[i]:init(x[i], g[i], tau, k_gain, d_gain, zero)
  end
end

function dmp.learn_minimum_jerk_trajectory(o, x, g, tau, nbasis, k_gain, d_gain)
  -- learn dmp parameters from minimum jerk trajectory
  -- x               : starting position
  -- g               : goal position
  -- tau             : duration
  -- nbasis          : optional number of basis functions
  -- k_gain          : optional spring constant
  -- d_gain          : optional damping coefficient

  local nbasis       = nbasis or 15
  local N            = 5000
  local tdata        = {}
  local xdata        = {}
  local minimum_jerk = {}

  -- compute minimum jerk trajectory given x, g and tau
  for i = 1,o.ndims do
    xdata[i] = {}
    minimum_jerk[i] = trajectory.minimum_jerk(x[i], g[i], tau)
  end

  for i = 1,N do
    tdata[i] = tau*(i - 1)/(N - 1)
    for j = 1,o.ndims do
      xdata[j][i] = (minimum_jerk[j])(tdata[i])
    end
  end

  return o:learn_trajectory(xdata, tdata, nbasis, k_gain, d_gain)
end

function dmp.learn_trajectory(o, xdata, tdata, nbasis, k_gain, d_gain)
  -- learn dmp parameters from time-series trajectory
  -- xdata           : position samples for each DoF  [ndims x N]
  -- tdata           : sample times for each position [1 x N]
  -- nbasis          : optional number of basis functions
  -- k_gain          : optional spring constant
  -- d_gain          : optional damping coefficient

  local x            = {}
  local xd           = {}
  local xdd          = {}
  local state        = {}
  local fdata        = {}
  local sdata        = {}
  local tau          = tdata[#tdata] - tdata[1]
  assert(#xdata == o.ndims, 'invalid xdata dimensions')

  -- initialize dynamical systems
  o.canonical_system:init(tau)
  for i = 1,o.ndims do
    local x0 = xdata[i][1]
    local g0 = xdata[i][#xdata[i]]
    o.transform_system[i]:init(x0, g0, tau, k_gain, d_gain, zero)
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

  -- calculate nonlinearities
  for i = 1,#tdata do
    local ipast = math.max(i - 1, 1)
    local inext = math.min(i + 1, #tdata)
    local dt = (tdata[inext] - tdata[ipast])/2
    local s = o.canonical_system:integrate(dt)
    for j = 1,o.ndims do
      state[1] = x[j][i]
      state[2] = xd[j][i]
      state[3] = xdd[j][i]
      fdata[j][i] = o.transform_system[j]:estimate_nonlinearity(s, state)
    end
    sdata[i] = s
  end

  -- initialize parameters for radial basis functions
  local centers = {}
  local widths  = {}
  local alpha = o.canonical_system.alpha
  for i = 1,nbasis do
    centers[i] = math.exp(-alpha*(i - 1)/(nbasis - 1))
    widths[i] = 0.5*(centers[i] - (centers[i-1] or 0))^(-2)
  end

  -- learn nonlinear functions via linear regression
  for i = 1,o.ndims do
    local f = dmp_nonlinearity.new(centers, widths)
    f:fit(fdata[i], sdata)
    o.transform_system[i]:set_nonlinearity(f)
  end
end

function dmp.integrate(o, coupling, dt)
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
----------------------------------------------------------------------

function dmp_nonlinearity.new(...)
  local o = {}
  local arg = {...}
  if #arg < 2 then
    o.nbasis = arg[1] or 1                  -- number of basis functions
    o.center = zeros(o.nbasis)              -- radial basis centers
    o.width = zeros(o.nbasis)               -- radial basis widths
    o.theta = zeros(o.nbasis)               -- radial basis weigths
  else
    o.nbasis = #arg[1]
    o.center = arg[1]
    o.width = arg[2]
    o.theta = arg[3] or zeros(o.nbasis)
  end
  return setmetatable(o, dmp_nonlinearity)
end

function dmp_nonlinearity.get_basis_centers(o)
  return copy_array(o.center)
end

function dmp_nonlinearity.get_basis_widths(o)
  return copy_array(o.width)
end

function dmp_nonlinearity.get_basis_weights(o)
  return copy_array(o.theta)
end

function dmp_nonlinearity.set_basis_centers(o, centers)
  o.center = copy_array(centers)
end

function dmp_nonlinearity.set_basis_widths(o, widths)
  o.width = copy_array(widths)
end

function dmp_nonlinearity.set_basis_weights(o, thetas)
  o.theta = copy_array(weights)
end

function dmp_nonlinearity.fit(o, fdata, sdata)
  -- calculate basis weights via ordinary least squares
  -- fdata           : sampled nonlinearity signal [1 x N]
  -- sdata           : phase values at each sample [1 x N]

  local y = matrix.fromtable(fdata)
  local X = matrix.new(#sdata, o.nbasis)

  for i = 1,#sdata do
    local psi = matrix.new(o.nbasis)
    for j = 1,o.nbasis do
      psi[j] = gaussian(sdata[i], o.center[j], o.width[j])
    end
    local sum = matrix.sum(psi)
    for j = 1,o.nbasis do
      X[i][j] = psi[j]*sdata[i]/sum
    end
  end

  o.theta = (X % y):totable()
end

function dmp_nonlinearity.predict(o, s)
  -- predict y = f(s) using linear regression
  -- s               : canonical system state (dmp phase)

  local f, sum = 0, 0
  for i = 1,o.nbasis do
    local psi = gaussian(s, o.center[i], o.width[i])
    f = f + o.theta[i]*psi
    sum = sum + psi
  end

  return f*s/sum
end

-- canonical system
----------------------------------------------------------------------

function dmp_canonical_system.new(tau, alpha)
  local o = {}
  dmp_canonical_system.init(o, tau, alpha)
  return setmetatable(o, dmp_canonical_system)
end

function dmp_canonical_system.init(o, tau, alpha)
  o.s     = 1                               -- phase
  o.tau   = tau or 1                        -- duration
  o.tau0  = tau or 1                        -- nominal duration
  o.alpha = alpha or -math.log(0.01)        -- spring constant 
  o.dt    = nil                             -- integrator time step
end

function dmp_canonical_system.set_time_step(o, dt)
  o.dt = dt
end

function dmp_canonical_system.reset(o, tau)
  o.s   = 1
  o.tau = tau or o.tau0
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
  -- dt : optional time_step 

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
----------------------------------------------------------------------

function dmp_transform_system.new(x, g, tau, k_gain, d_gain, f)
  local o = {}
  dmp_transform_system.init(o, x, g, tau, k_gain, d_gain, f)
  return setmetatable(o, dmp_transform_system)
end

function dmp_transform_system.init(o, x, g, tau, k_gain, d_gain, f)
  o.x       = x or 0                          -- start position
  o.x0      = x or 0                          -- nominal start position
  o.g       = g or 0                          -- goal position
  o.g0      = g or 0                          -- nominal goal position
  o.tau     = tau or 1                        -- duration
  o.tau0    = tau or 1                        -- nominal duration
  o.state   = {o.x, 0, 0}                     -- state {x, xd, xdd}
  o.k_gain  = k_gain or 250                   -- spring constant
  o.d_gain  = d_gain or 2*math.sqrt(o.k_gain) -- damping coefficient
  o.f       = f or dmp_nonlinearity.new()     -- nonlinear function f(s)
  o.dt      = nil                             -- integrator time step
end

function dmp_transform_system.set_time_step(o, dt)
  o.dt = dt
end

function dmp_transform_system.set_nonlinearity(o, f)
  o.f = f
end

function dmp_transform_system.reset(o, x, g, tau)
  o.x   = x or o.x0 
  o.g   = g or o.g0
  o.tau = tau or o.tau0
end

function dmp_transform_system.set_start_position(o, x)
  o.x = x
end

function dmp_transform_system.set_goal_position(o, g)
  o.g = g
end

function dmp_transform_system.set_duration(o, tau)
  o.tau = tau
end

function dmp_transform_system.set_state(o, state)
  o.state[1] = state[1]
  o.state[2] = state[2]
  o.state[3] = state[3]
end

function dmp_transform_system.get_nonlinearity(o)
  return o.f
end

function dmp_transform_system.get_start_position(o)
  return o.x
end

function dmp_transform_system.get_goal_position(o)
  return o.g
end

function dmp_transform_system.get_duration(o)
  return o.tau
end

function dmp_transform_system.get_state(o)
  return copy_array(state)
end

function dmp_transform_system.estimate_nonlinearity(o, s, state)
  -- estimate the nonlinearity f(s) given the desired phase / state
  -- s             : canonical system state (dmp phase)
  -- state         : desired system state {x, xd, xdd}

  local fs
  local x, xd, xdd = unpack(state)

  fs = (xdd*o.tau^2 + o.d_gain*(xd*o.tau))/o.k_gain
     - (o.g - x) + (o.g - o.x)*s

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
      - o.k_gain*(o.g - o.x)*s + o.k_gain*fs
  xdd = xdd/o.tau^2 + coupling
  xd  = xd + 1/2*xdd*dt

  o.state[1] = x
  o.state[2] = xd
  o.state[3] = xdd
  return {x, xd, xdd} 
end

return dmp
