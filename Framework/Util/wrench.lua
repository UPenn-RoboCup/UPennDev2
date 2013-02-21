require('vector')

----------------------------------------------------------------------
-- wrench : 6 dof spatial force vector
----------------------------------------------------------------------

wrench = {}
wrench.__index = wrench
wrench.__mtstring = 'wrench'

function wrench.new(w)
  w = w or {}
  for i = #w,6 do
    w[i] = 0
  end
  return setmetatable(w, wrench)
end

function wrench.ones()
  w = {}
  for i = 1,6 do
    w[i] = 1
  end
  return setmetatable(w, wrench)
end

function wrench.zeros(n)
  w = {}
  for i = 1,6 do
    w[i] = 0
  end
  return setmetatable(w, wrench)
end

function wrench.set_force(w, v)
  w[1] = v[1]
  w[2] = v[2]
  w[3] = v[3]
end

function wrench.set_torque(w, v)
  w[4] = v[1]
  w[5] = v[2]
  w[6] = v[3]
end

function wrench.get_force(w)
  return vector.new({w[1], w[2], w[3]})
end

function wrench.get_torque(w)
  return vector.new({w[4], w[5], w[6]})
end

function wrench.copy(w1)
  if not w1 then return nil end
  local w = {}
  for i = 1,6 do
    w[i] = w1[i]
  end
  return setmetatable(w, wrench)
end

function wrench.translate(w1, origin)
  -- get equivalent wrench at relative origin
  local w = {}
  local o = origin
  local fx, fy, fz = w1[1], w1[2], w1[3]
  local tx, ty, tz = w1[4], w1[5], w1[6]
  local rx, ry, rz =  o[1],  o[2],  o[3]
  w[1] = fx
  w[2] = fy
  w[3] = fz
  w[4] = tx - fz*ry + fy*rz
  w[5] = ty - fx*rz + fz*rx
  w[6] = tz - fy*rx + fx*ry
  return setmetatable(w, wrench) 
end

function wrench.norm(w1)
  local s = 0
  for i = 1,6 do
    s = s + w1[i]*w1[i]
  end
  return math.sqrt(s)
end

function wrench.mulnum(w1, a)
  local w = {}
  for i = 1,6 do
    w[i] = a*w1[i]
  end
  return setmetatable(w, wrench)
end

function wrench.divnum(w1, a)
  local w = {}
  for i = 1,6 do
    w[i] = w1[i]/a
  end
  return setmetatable(w, wrench)
end

function wrench.__add(w1, w2)
  if getmetatable(w1) == getmetatable(w2) then 
    local w = {}
    for i = 1,6 do
      w[i] = w1[i] + w2[i]
    end
    return setmetatable(w, wrench)
  else
    error("attempt to add wrench to non-wrench type")
  end
end

function wrench.__sub(w1, w2)
  if getmetatable(w1) == getmetatable(w2) then 
    local w = {}
    for i = 1,6 do
      w[i] = w1[i] - w2[i]
    end
    return setmetatable(w, wrench)
  else
    error("attempt to subtract non-wrench type from wrench")
  end
end

function wrench.__mul(w1, w2)
  if type(w2) == 'number' then
    return wrench.mulnum(w1, w2)
  elseif type(w1) == 'number' then
    return wrench.mulnum(w2, w1)
  else
    local s = 0
    for i = 1,#w1 do
      s = s + w1[i]*w2[i]
    end
    return s
  end
end

function wrench.__div(w1, w2)
  if type(w2) == 'number' then
    return wrench.divnum(w1, w2)
  else
    error("attempt to divide wrench by non-number type")
  end
end

function wrench.__unm(w1)
  return wrench.mulnum(w1, -1)
end

function wrench.__tostring(w1)
  return "{"..table.concat(w1, ', ').."}"
end

return wrench
