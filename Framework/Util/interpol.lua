interpol = {}

function interpol.linear_segment(p0, p1, t0, t1)
  -- linear interpolation on the interval t = (t0, t1)
  local t0 = t0 or 0 
  local t1 = t1 or 1 
  return function (t)
    local t = (t - t0)/(t1 - t0)
    if (t < 0) then t = 0 end
    if (t > 1) then t = 1 end
    return p0 + (p1 - p0)*t
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
