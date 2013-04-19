--------------------------------------------------------------------------------
-- trajectory : utilities for online and offline trajectory generation
--------------------------------------------------------------------------------

trajectory = {}

-- utilities
--------------------------------------------------------------------------------

local function minimum_jerk_coefficients(x0, x1)
  -- returns 'b_k' coefficients for minimum jerk trajectory computation
  -- see "Jerk-Bounded Manipulator Trajectory Planning: Design for 
  -- Real-Time Applications," S. Macfarlane and E. Croft

  local x0, xd0, xdd0 = unpack(x0)
  local x1, xd1, xdd1 = unpack(x1) 

  local b = {}
  b[1] = x0
  b[2] = xd0
  b[3] = 0.5*xdd0
  b[4] = -10*x0 - 6*xd0 - 1.5*xdd0 + 10*x1 - 4*xd1 + 0.5*xdd1
  b[5] =  15*x0 + 8*xd0 + 1.5*xdd0 - 15*x1 + 7*xd1 - xdd1
  b[6] =  -6*x0 - 3*xd0 - 0.5*xdd0 + 6*x1 - 3*xd1 + 0.5*xdd1
  return b
end

-- offline trajectory generators
--------------------------------------------------------------------------------

function trajectory.linear(x0, x1, tau)
  -- returns linear trajectory on the interval t = (0, tau)
  return function (t)
    local t = t/tau
    if (t < 0) then t = 0 end
    if (t > 1) then t = 1 end
    return x0 + (x1 - x0)*t
  end
end

function trajectory.hermite_curve(x0, x1, tau)
  -- returns hermite cubic spline trajectory on the interval t = (0, tau)
  local x0, xd0 = unpack(x0)
  local x1, xd1 = unpack(x1)
  xd0 = xd0 and tau*xd0 or 0
  xd1 = xd1 and tau*xd1 or 0

  return function (t)
    local t = t/tau
    if (t < 0) then t = 0 end
    if (t > 1) then t = 1 end
    local t2, t3 = t^2, t^3
    return (2*t3 - 3*t2 + 1)*x0
         + (t3 - 2*t2 + t)*xd0
         + (-2*t3 + 3*t2)*x1
         + (t3 - t2)*xd1
  end
end

function trajectory.minimum_jerk(x0, x1, tau)
  -- returns minimum jerk trajectory on the interval t = (0, tau)
  if (type(x0) == 'number') then x0 = {x0} end
  if (type(x1) == 'number') then x1 = {x1} end
  x0[2], x0[3] = x0[2] or 0, x0[3] or 0
  x1[2], x1[3] = x1[2] or 0, x1[3] or 0

  if (tau <= 0) then
    return function (t) return unpack(x1) end
  end

  x0[2], x0[3] = tau*x0[2], tau^2*x0[3]
  x1[2], x1[3] = tau*x1[2], tau^2*x1[3]

  local b = minimum_jerk_coefficients(x0, x1)

  return function (t)
    local t = t/tau
    if (t < 0) then t = 0 end
    if (t > 1) then t = 1 end
    local x = 0
    local xd = 0
    local xdd = 0 
    for k = 0,5 do
      x = x + b[k + 1]*t^k
    end
    for k = 1,5 do
      xd = xd + k*b[k + 1]*t^(k - 1)
    end
    for k = 2,5 do
      xdd = xdd + k*(k - 1)*b[k + 1]*t^(k - 2)
    end
    return x, xd/tau, xdd/tau^2
  end
end

-- online trajectory generators
--------------------------------------------------------------------------------

function trajectory.minimum_jerk_step(x0, x1, tau, dt)
  -- returns x, xd and xdd at time t + dt along minimum jerk trajectory
  if (type(x0) == 'number') then x0 = {x0} end
  if (type(x1) == 'number') then x1 = {x1} end
  x0[2], x0[3] = x0[2] or 0, x0[3] or 0
  x1[2], x1[3] = x1[2] or 0, x1[3] or 0

  if (tau <= 0) then
    return unpack(x1)
  end

  x0[2], x0[3] = tau*x0[2], tau^2*x0[3]
  x1[2], x1[3] = tau*x1[2], tau^2*x1[3]

  local b = minimum_jerk_coefficients(x0, x1)

  local dt = dt/tau
  if (dt < 0) then dt = 0 end
  if (dt > 1) then dt = 1 end
  local x = 0
  local xd = 0
  local xdd = 0 
  for k = 0,5 do
    x = x + b[k + 1]*dt^k
  end
  for k = 1,5 do
    xd = xd + k*b[k + 1]*dt^(k - 1)
  end
  for k = 2,5 do
    xdd = xdd + k*(k - 1)*b[k + 1]*dt^(k - 2)
  end
  return x, xd/tau, xdd/tau^2
end

return trajectory
