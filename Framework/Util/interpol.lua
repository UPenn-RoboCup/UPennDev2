require('numlua')

----------------------------------------------------------------------
-- interpol : utilities for interpolation and trajectory generation
----------------------------------------------------------------------

interpol = {}

function interpol.linear_segment(x0, x1, t0, t1)
  -- linear interpolation on the interval t = (t0, t1)
  local t0 = t0 or 0 
  local t1 = t1 or 1 

  return function (t)
    local t = (t - t0)/(t1 - t0)
    if (t < 0) then t = 0 end
    if (t > 1) then t = 1 end
    return x0 + (x1 - x0)*t
  end
end

function interpol.minimum_jerk_trajectory(x0, x1, t0, t1)
  -- minimum jerk trajectory on the interval t = (t0, t1)
  if (type(x0) == 'number') then x0 = {x0} end
  if (type(x1) == 'number') then x1 = {x1} end

  local x0, dx0, ddx0 = unpack(x0)
  local x1, dx1, ddx1 = unpack(x1)
  dx0, ddx0 = dx0 or 0, ddx0 or 0
  dx1, ddx1 = dx1 or 0, ddx1 or 0
  local T   = t1 - t0
  local T2  = T^2
  local T3  = T^3
  local T4  = T^4
  local T5  = T^5

  local a = matrix.zeros(6)
  a[1] = x0
  a[2] = dx0
  a[3] = ddx0/2

  local b = {}
  b[1] = {   T3,    T4,    T5} 
  b[2] = { 3*T2,  4*T4,  5*T5}
  b[3] = {  6*T, 12*T2, 20*T3}
  b = matrix.fromtable(b)

  local c = matrix.zeros(3)
  c[1] = x1 - a[1] - a[2] - a[3]
  c[2] = dx1 - a[2] - 2*a[3]
  c[3] = ddx1 - 2*a[3]
                              
  a(4,6)['_']  = b:inv()*c

  return function (t)
    local x = 0
    local dx = 0
    local ddx = 0 
    if (t < t0) then t = t0 end
    if (t > t1) then t = t1 end
    for k = 0,5 do
      x   = x + a[k + 1]*(t - t0)^k
      dx  = dx + k*a[k + 1]*(t - t0)^(k - 1)
      ddx = ddx + k*(k - 1)*a[k + 1]*(t - t0)^(k - 2)
    end
    return x, dx, ddx
  end
end

function interpol.hermite_curve(p0, p1, m0, m1, t0, t1)
  -- hermite cubic spline interpolation on the interval t = (t0, t1)
  local t0 = t0 or 0 
  local t1 = t1 or 1 
  local m0 = m0 and (t1 - t0)*m0 or 0
  local m1 = m1 and (t1 - t0)*m1 or 0

  return function (t)
    local t = (t - t0)/(t1 - t0)
    if (t < 0) then t = 0 end
    if (t > 1) then t = 1 end
    local t2, t3 = t*t, t*t*t
    return (2*t3 - 3*t2 + 1)*p0
         + (t3 - 2*t2 + t)*m0
         + (-2*t3 + 3*t2)*p1
         + (t3 - t2)*m1
  end
end

return interpol
