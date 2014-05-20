local vector  = {}
local mt      = {}
local mt_pose = {}

local function new(t)
  local ty = type(t)
  if type(t)=='number' then
    t = {t}
  elseif ty=='userdata' then
    local n = #t
    if type(n)~='number' then n = n[1] end
    local tt = {}
    for i=1,n do tt[i] = t[i] end
    t = tt
  end
  t = t or {}
  return setmetatable(t, mt)
end

local function copy(t)
  local tt = {}
  for i=1,#t do tt[i] = t[i] end
  return setmetatable(t, mt)
end

local function ones(n)
  n = n or 1
  local t = {}
  for i = 1, n do t[i] = 1 end
  return setmetatable(t, mt)
end

local function zeros(n)
  n = n or 1
  local t = {}
  for i = 1, n do t[i] = 0 end
  return setmetatable(t, mt)
end

local function count(start,n)
  n = n or 1
  local t = {}
  for i = 1,n do t[i] = start+i-1 end
  return setmetatable(t, mt)
end

local function slice(v1, istart, iend)
  local v = {}
  istart = istart or 1
  iend = iend or #v1
  for i = 1,iend-istart+1 do
    v[i] = v1[istart+i-1] or (0 / 0)
  end
  return setmetatable(v, mt)
end

local function add(v1, v2)
  local v = {}
  for i = 1, #v1 do v[i] = v1[i] + v2[i] end
  return setmetatable(v, mt)
end

local function sub(v1, v2)
  local v = {}
  for i = 1, #v1 do v[i] = v1[i] - v2[i] end
  return setmetatable(v, mt)
end

local function mulnum(v1, a)
  local v = {}
  for i = 1, #v1 do v[i] = a * v1[i] end
  return setmetatable(v, mt)
end

local function divnum(v1, a)
  local v = {}
  for i = 1, #v1 do v[i] = v1[i] / a end
  return setmetatable(v, mt)
end

local function mul(v1, v2)
  if type(v2) == "number" then
    return mulnum(v1, v2)
  elseif type(v1) == "number" then
    return mulnum(v2, v1)
  else
    local s = 0
    for i = 1, #v1 do s = s + v1[i] * v2[i] end
    return s
  end
end

local function unm(v1)
  return mulnum(v1, -1)
end

local function eq(v1,v2)
  for i,v in ipairs(v1) do
    if v~=v2[i] then return false end
  end
  return true
end

local function div(v1, v2)
  if type(v2) == "number" then
    return divnum(v1, v2)
  else
    return nil
  end
end

local function norm(v1)
  local s = 0
  for i = 1, #v1 do s = s + v1[i] * v1[i] end
  return math.sqrt(s)
end

local function sum(v1)
  local s = 0
  for i = 1, #v1 do s = s + v1[i] end
  return s
end

local function cross(v1,v2)
	local v = {}
  v[1] =   ( (v1[2] * v2[3]) - (v1[3] * v2[2]) )
  v[2] = - ( (v1[1] * v2[3]) - (v1[3] * v2[1]) )
  v[3] =   ( (v1[1] * v2[2]) - (v1[2] * v2[1]) )
  return setmetatable(v, mt)
end

local function tostring(v1, formatstr)
  formatstr = formatstr or "%g"
  local str = "{"..string.format(formatstr, v1[1] or 0/0)
  for i = 2, #v1 do
    str = str..", "..string.format(formatstr,v1[i])
  end
  str = str.."}"
  return str
end

-- Metatables for pose vectors
-- TODO: Use as a utility pose file, too
local function pose_index(p,idx)
  if idx=='x' then
    return p[1]
  elseif idx=='y' then
    return p[2]
  elseif idx=='a' then
    return p[3]
  end
end

local function pose_newindex(p,idx,val)
  if idx=='x' then
    p[1] = val
  elseif idx=='y' then
    p[2] = val
  elseif idx=='a' then
    p[3] = val
  end
end

local function pose_tostring(p)
  return string.format(
    "{x=%g, y=%g, a=%g degrees}",
    p[1], p[2], p[3]*180/math.pi
  )
end

local function pose(t)
  if type(t)=='table' and #t>=3 then
    -- good pose
    return setmetatable(t, mt_pose)
  end
  return setmetatable({0,0,0}, mt_pose)
end

-- Regular vector
mt.__eq  = eq
mt.__add = add
mt.__sub = sub
mt.__mul = mul
mt.__div = div
mt.__unm = unm
mt.__tostring = tostring

-- Pose vector
mt_pose.__add = add
mt_pose.__sub = sub
mt_pose.__mul = mul
mt_pose.__div = div
mt_pose.__unm = unm
mt_pose.__index    = pose_index
mt_pose.__newindex = pose_newindex
mt_pose.__tostring = pose_tostring

-- Vector object (so there is no module use)
-- constructors
vector.new   = new
vector.zeros = zeros
vector.ones  = ones
vector.count = count
vector.pose  = pose
-- special metatable
vector.add = add
vector.sub = sub
vector.mul = mul
vector.div = div
vector.unm = unm
-- should be in metatable...
vector.slice = slice
vector.norm  = norm
vector.cross = cross
vector.copy = copy
vector.sum = sum

return vector
