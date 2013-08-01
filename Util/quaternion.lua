local quaternion = {}

local mt = {}

local function norm(v1)
  local s = 0
  for i = 1, #v1 do s = s + v1[i] * v1[i] end
  return math.sqrt(s)
end

-- New quaternion from a 3 or 4 element table
function new(t,alpha)
  if not t then return setmetatable({1,0,0,0}, mt) end
  if #t==4 then return setmetatable(t, mt) end
  -- Rotation vector to a quaternion
  assert(#t==3,'Must give a 3 or 4 element table to make a quaternion')
  local mag = quaternion.norm(t)
  alpha = alpha or mag
  local scale = math.sin(alpha/2) / mag
  t = { math.cos(alpha/2), scale*t[1], scale*t[2], scale*t[3] }
  return setmetatable(t, mt)
end

function conjugate(v1, istart, iend)
  local v = {}
  iend = iend or #v1
  for i = 1,iend-istart+1 do
    v[i] = v1[istart+i-1]
  end
  return setmetatable(v, mt)
end

function add(v1, v2)
  local v = {}
  for i = 1, #v1 do v[i] = v1[i] + v2[i] end
  return setmetatable(v, mt)
end

function sub(v1, v2)
  local v = {}
  for i = 1, #v1 do
    v[i] = v1[i] - v2[i]
  end
  return setmetatable(v, mt)
end

function divnum(v1, a)
  local v = {}
  for i = 1, #v1 do
    v[i] = v1[i]/a
  end
  return setmetatable(v, mt)
end

function mul(v1, v2)
  local s = 0
  for i = 1, #v1 do
    s = s + v1[i] * v2[i]
  end
  return s
end

function div(v1, v2)
  if type(v2) == "number" then
    return divnum(v1, v2)
  else
    return nil
  end
end


local function tostring(v1, formatstr)
  formatstr = formatstr or "%g"
  local str = "{"..string.format(formatstr, v1[1])
  for i = 2, #v1 do
    str = str..", "..string.format(formatstr,v1[i])
  end
  str = str.."}"
  return str
end

mt.__add = add
mt.__sub = sub
mt.__mul = mul
mt.__div = div
mt.__unm = unm
mt.__tostring = tostring

