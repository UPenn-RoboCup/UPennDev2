local vector  = {}
local mt      = {}

function vector.new(t)
  local ty = type(t)
  if type(t)=='number' then
    t = {t}
  elseif ty=='userdata' then
    local n = #t
    if type(n)~='number' then n = n[1] end
    local tt = {}
    for i=1,n do tt[i] = t[i] end
    t = tt
	elseif ty~='table' then
		t = {}
  end
  return setmetatable(t, mt)
end
function vector.copy(t, tt)
  tt = tt or {}
  for i=1,#t do tt[i] = t[i] end
  return setmetatable(tt, mt)
end

function vector.ones(n)
  local t = {}
  for i = 1,(n or 1) do t[i] = 1 end
  return setmetatable(t, mt)
end

function vector.zeros(n)
  local t = {}
  for i = 1, (n or 1) do t[i] = 0 end
  return setmetatable(t, mt)
end

function vector.count(start, n)
  local t = {}
  for i = 1,(n or 1) do t[i] = start+i-1 end
  return setmetatable(t, mt)
end

function vector.slice(v1, istart, iend)
  local v = {}
  istart = istart or 1
  iend = iend or #v1
	if istart==iend then return v1[istart] end
  for i = 1,iend-istart+1 do
    v[i] = v1[istart+i-1] or (0 / 0)
  end
  return setmetatable(v, mt)
end

local sqrt = require'math'.sqrt
local pow = require'math'.pow
function vector.norm(v1)
  local s = 0
  for i = 1, #v1 do s = s + pow(v1[i], 2) end
  return sqrt(s)
end

function vector.sum(v1)
  local s = 0
  for i = 1, #v1 do s = s + v1[i] end
  return s
end

function vector.contains(v1, num)
  for i, v in ipairs(v1) do
    if v==num then return true end
  end
  return false
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

local function eq(v1, v2)
	if #v1~=#v2 then return false end
  for i,v in ipairs(v1) do
    if v~=v2[i] then return false end
  end
  return true
end

local function div(v1, v2)
  if type(v2) == "number" then
    return divnum(v1, v2)
	else
		-- pointwise
		local v = {}
		for i,val in ipairs(v1) do v[i] = val / v2[i] end
		return setmetatable(v, mt)
  end
end

local function v_tostring(v1, formatstr)
  formatstr = formatstr or "%g"
  local str = "{"..string.format(formatstr, v1[1] or 0/0)
  for i = 2, #v1 do
    str = str..", "..string.format(formatstr,v1[i])
  end
  str = str.."}"
  return str
end

----[[
-- Metatables for pose vectors
local mt_pose = {}
-- TODO: Use as a utility pose file, too
local function pose_index(p, idx)
  if idx=='x' then
    return p[1]
  elseif idx=='y' then
    return p[2]
  elseif idx=='a' then
    return p[3]
  end
end

local function pose_newindex(p, idx, val)
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

function vector.pose(t)
  if type(t)=='table' and #t>=3 then
    -- good pose
    return setmetatable(t, mt_pose)
  end
  return setmetatable({0,0,0}, mt_pose)
end

-- Pose vector
mt_pose.__add = add
mt_pose.__sub = sub
mt_pose.__mul = mul
mt_pose.__div = div
mt_pose.__unm = unm
mt_pose.__index    = pose_index
mt_pose.__newindex = pose_newindex
mt_pose.__tostring = pose_tostring
--]]

-- Regular vector
mt.__eq  = eq
mt.__add = add
mt.__sub = sub
mt.__mul = mul
mt.__div = div
mt.__unm = unm
mt.__tostring = v_tostring

return vector
