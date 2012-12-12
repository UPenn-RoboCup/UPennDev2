require('numlua')

----------------------------------------------------------------------
-- trajectory : utilities for online and offline trajectory generation
----------------------------------------------------------------------

trajectory = {}

function trajectory.offline_linear(x0, x1, tau)
  -- returns linear trajectory on the interval t = (0, tau)
  return function (t)
    local t = t/tau
    if (t < 0) then t = 0 end
    if (t > 1) then t = 1 end
    return x0 + (x1 - x0)*t
  end
end

function trajectory.offline_hermite_curve(x0, x1, tau)
  -- returns hermite cubic spline trajectory on the interval t = (0, tau)
  local x0, xd0 = unpack(x0)
  local x1, xd1 = unpack(x1)
  xd0 = xd0 and tau*xd0 or 0
  xd1 = xd1 and tau*xd1 or 0

  return function (t)
    local t = t/tau
    if (t < 0) then t = 0 end
    if (t > 1) then t = 1 end
    local t2, t3 = t*t, t*t*t
    return (2*t3 - 3*t2 + 1)*x0
         + (t3 - 2*t2 + t)*xd0
         + (-2*t3 + 3*t2)*x1
         + (t3 - t2)*xd1
  end
end

function trajectory.offline_minimum_jerk(x0, x1, tau)
  -- returns minimum jerk trajectory on the interval t = (0, tau)
  -- reference: "Minimum Jerk Trajectories", Javier R. Movellan
  if (type(x0) == 'number') then x0 = {x0} end
  if (type(x1) == 'number') then x1 = {x1} end

  local x0, xd0, xdd0 = unpack(x0)
  local x1, xd1, xdd1 = unpack(x1)
  xd0, xdd0 = xd0 or 0, xdd0 or 0
  xd1, xdd1 = xd1 or 0, xdd1 or 0
  local T   = tau
  local T2  = T^2
  local T3  = T^3
  local T4  = T^4
  local T5  = T^5

  local a = matrix.zeros(6)
  a[1] = x0
  a[2] = xd0
  a[3] = xdd0/2

  local b = {}
  b[1] = {   T3,    T4,    T5} 
  b[2] = { 3*T2,  4*T4,  5*T5}
  b[3] = {  6*T, 12*T2, 20*T3}
  b = matrix.fromtable(b)

  local c = matrix.zeros(3)
  c[1] = x1 - a[1] - a[2] - a[3]
  c[2] = xd1 - a[2] - 2*a[3]
  c[3] = xdd1 - 2*a[3]
                              
  a(4,6)['_']  = b:inv()*c

  return function (t)
    local x = 0
    local xd = 0
    local xdd = 0 
    if (t < 0) then t = 0 end
    if (t > T) then t = T end
    for k = 0,5 do
      x   = x + a[k + 1]*t^k
      xd  = xd + k*a[k + 1]*t^(k - 1)
      xdd = xdd + k*(k - 1)*a[k + 1]*t^(k - 2)
    end
    return x, xd, xdd
  end
end

return trajectory
