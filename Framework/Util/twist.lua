require('vector')

----------------------------------------------------------------------
-- twist : 6 dof spatial velocity vector 
----------------------------------------------------------------------

twist = {}
twist.__index = twist
twist.__mtstring = 'twist'

function twist.new(t)
  t = t or {}
  for i = #t,6 do
    t[i] = 0
  end
  return setmetatable(t, twist)
end

function twist.ones()
  t = {}
  for i = 1,6 do
    t[i] = 1
  end
  return setmetatable(t, twist)
end

function twist.zeros(n)
  t = {}
  for i = 1,6 do
    t[i] = 0
  end
  return setmetatable(t, twist)
end

function twist.set_lin(t, v)
  t[1] = v[1]
  t[2] = v[2]
  t[3] = v[3]
end

function twist.set_rot(t, v)
  t[4] = v[1]
  t[5] = v[2]
  t[6] = v[3]
end

function twist.get_lin(t)
  return vector.new({t[1], t[2], t[3]})
end

function twist.get_rot(t)
  return vector.new({t[4], t[5], t[6]})
end

function twist.copy(t1)
  if not t1 then return nil end
  local t = {}
  for i = 1,6 do
    t[i] = t1[i]
  end
  return setmetatable(t, twist)
end

function twist.translate(t1, origin)
  -- get equivalent twist at relative origin
  local t = {}
  local o = origin
  local vx, vy, vz = t1[1], t1[2], t1[3]
  local wx, wy, wz = t1[4], t1[5], t1[6]
  local rx, ry, rz =  o[1],  o[2],  o[3]
  t[1] = vx - wz*ry + wy*rz
  t[2] = vy - wx*rz + wz*rx
  t[3] = vz - wy*rx + wx*ry
  t[4] = wx
  t[5] = wy
  t[6] = wz
  return setmetatable(t, twist) 
end

function twist.norm(t1)
  local s = 0
  for i = 1,6 do
    s = s + t1[i]*t1[i]
  end
  return math.sqrt(s)
end

function twist.mulnum(t1, a)
  local t = {}
  for i = 1,6 do
    t[i] = a*t1[i]
  end
  return setmetatable(t, twist)
end

function twist.divnum(t1, a)
  local t = {}
  for i = 1,6 do
    t[i] = t1[i]/a
  end
  return setmetatable(t, twist)
end

function twist.__add(t1, t2)
  if getmetatable(t1) == getmetatable(t2) then 
    local t = {}
    for i = 1,6 do
      t[i] = t1[i] + t2[i]
    end
    return setmetatable(t, twist)
  else
    error("attempt to add twist to non-twist type")
  end
end

function twist.__sub(t1, t2)
  if getmetatable(t1) == getmetatable(t2) then 
    local t = {}
    for i = 1,6 do
      t[i] = t1[i] - t2[i]
    end
    return setmetatable(t, twist)
  else
    error("attempt to subtract non-twist type from twist")
  end
end

function twist.__mul(t1, t2)
  if type(t2) == 'number' then
    return twist.mulnum(t1, t2)
  elseif type(t1) == 'number' then
    return twist.mulnum(t2, t1)
  else
    local s = 0
    for i = 1,#t1 do
      s = s + t1[i]*t2[i]
    end
    return s
  end
end

function twist.__div(t1, t2)
  if type(t2) == 'number' then
    return twist.divnum(t1, t2)
  else
    error("attempt to divide twist by non-number type")
  end
end

function twist.__unm(t1)
  return twist.mulnum(t1, -1)
end

function twist.__tostring(t1)
  return "{"..table.concat(t1, ', ').."}"
end

return twist
