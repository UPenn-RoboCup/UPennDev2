local vector = require'vector'
local quaternion = require'quaternion'
local Transform = {}

mt = {}

function Transform.inv(a)
  local t = {}
  local r = {}
  local p = {}
  r[1] = vector.new({a[1][1],a[2][1],a[3][1]})
  r[2] = vector.new({a[1][2],a[2][2],a[3][2]})
  r[3] = vector.new({a[1][3],a[2][3],a[3][3]})
  p = vector.new({a[1][4],a[2][4],a[3][4]})
  t[1] = vector.new({r[1][1],r[1][2],r[1][3],-(r[1][1]*p[1]+r[1][2]*p[2]+r[1][3]*p[3])})
  t[2] = vector.new({r[2][1],r[2][2],r[2][3],-(r[2][1]*p[1]+r[2][2]*p[2]+r[2][3]*p[3])})
  t[3] = vector.new({r[3][1],r[3][2],r[3][3],-(r[3][1]*p[1]+r[3][2]*p[2]+r[3][3]*p[3])})
  t[4] = vector.new({0,0,0,1})
  return setmetatable(t,mt)
end

function Transform.eye()
  local t = {}
  t[1] = vector.new({1, 0, 0, 0})
  t[2] = vector.new({0, 1, 0, 0})
  t[3] = vector.new({0, 0, 1, 0})
  t[4] = vector.new({0, 0, 0, 1})
  return setmetatable(t, mt)
end

function Transform.rotZ(a)
  local ca = math.cos(a)
  local sa = math.sin(a)
  local t = {}
  t[1] = vector.new({ca, -sa, 0, 0})
  t[2] = vector.new({sa, ca, 0, 0})
  t[3] = vector.new({0, 0, 1, 0})
  t[4] = vector.new({0, 0, 0, 1})
  return setmetatable(t, mt)
end

function Transform.rotY(a)
  local ca = math.cos(a)
  local sa = math.sin(a)
  local t = {}
  t[1] = vector.new({ca, 0, sa, 0})
  t[2] = vector.new({0, 1, 0, 0})
  t[3] = vector.new({-sa, 0, ca, 0})
  t[4] = vector.new({0, 0, 0, 1})
  return setmetatable(t, mt)
end

function Transform.rotX(a)
  local ca = math.cos(a)
  local sa = math.sin(a)
  local t = {}
  t[1] = vector.new({1, 0, 0, 0})
  t[2] = vector.new({0, ca, -sa, 0})
  t[3] = vector.new({0, sa, ca, 0})
  t[4] = vector.new({0, 0, 0, 1})
  return setmetatable(t, mt)
end

function Transform.trans(dx, dy, dz)
  local t = {}
  t[1] = vector.new({1, 0, 0, dx})
  t[2] = vector.new({0, 1, 0, dy})
  t[3] = vector.new({0, 0, 1, dz})
  t[4] = vector.new({0, 0, 0, 1})
  return setmetatable(t, mt)
end

-- From NSL
-- I think this is wrong...
--http://www.gregslabaugh.name/publications/euler.pdf

--[[
function Transform.getEuler(t)
--returns euler angle (X,Y,Z) from rotation matrix
--Rotation sequence is Roll-Pitch-Yaw (rotY-rotX-rotZ)
   local e=vector.zeros(3)
   e[1]=-math.asin(t[3][2])
   e[2]=-math.atan2(-t[3][1],t[3][3])
   e[3]=-math.atan2(-t[1][2],t[2][2])
   return e
end
--]]

function Transform.getRPY(t)
  -- http://planning.cs.uiuc.edu/node103.html
  -- returns [roll, pitch, yaw] vector
  local e = vector.zeros(3)
  e[1]=math.atan2(t[3][2],t[3][3]) --Roll
  e[2]=math.atan2(-t[3][1],math.sqrt( t[3][2]^2 + t[3][3]^2) ) -- Pitch
  e[3]=math.atan2(t[2][1],t[1][1]) -- Yaw
  return e
end

function Transform.position6D(tr)
  local p = vector.new({
	tr[1][4],tr[2][4],tr[3][4],0,0,0})
  p[4] = math.atan2(tr[3][2],tr[3][3])
  p[5] = -math.asin(tr[3][1])
  p[6] = math.atan2(tr[2][1],tr[1][1])
  return p
end

-- Rotation Matrix to quaternion
-- from Yida
Transform.to_quaternion = function( R )
  local q = quaternion.new()
  local tr = R[1][1] + R[2][2] + R[3][3]
  if tr > 0 then
    local S = math.sqrt(tr + 1.0) * 2
    q[1] = 0.25 * S
    q[2] = (R[3][2] - R[2][3]) / S
    q[3] = (R[1][3] - R[3][1]) / S
    q[4] = (R[2][1] - R[1][2]) / S
  elseif R[1][1] > R[2][2] and R[1][1] > R[3][3] then
    local S = math.sqrt(1.0 + R[1][1] - R[2][2] - R[3][3]) * 2
    q[1] = (R[3][2] - R[2][3]) / S
    q[2] = 0.25 * S
    q[3] = (R[1][2] + R[2][1]) / S 
    q[4] = (R[1][3] + R[3][1]) / S
  elseif R[2][2] > R[3][3] then
    local S = math.sqrt(1.0 + R[2][2] - R[1][1] - R[3][3]) * 2
    q[1] = (R[1][3] - R[3][1]) / S
    q[2] = (R[1][2] + R[2][1]) / S 
    q[3] = 0.25 * S
    q[4] = (R[2][3] + R[3][2]) / S
  else
    local S = math.sqrt(1.0 + R[3][3] - R[1][1] - R[2][2]) * 2
    q[1] = (R[2][1] - R[1][2]) / S
    q[2] = (R[1][3] + R[3][1]) / S 
    q[3] = (R[2][3] + R[3][2]) / S
    q[4] = 0.25 * S
  end
  return q
end

function Transform.transform6D(p)
  local t = {}

  local cwx = math.cos(p[4])
  local swx = math.sin(p[4])
  local cwy = math.cos(p[5])
  local swy = math.sin(p[5])
  local cwz = math.cos(p[6])
  local swz = math.sin(p[6])

  t[1] = vector.new({1, 0, 0, 0})
  t[2] = vector.new({0, 1, 0, 0})
  t[3] = vector.new({0, 0, 1, 0})
  t[4] = vector.new({0, 0, 0, 1})

  t[1][1] = cwy*cwz
  t[1][2] = swx*swy*cwz-cwx*swz
  t[1][3] = cwx*swy*cwz+swx*swz
  t[1][4] = p[1]
  t[2][1] = cwy*swz
  t[2][2] = swx*swy*swz+cwx*cwz
  t[2][3] = cwx*swy*swz-swx*cwz
  t[2][4] = p[2]
  t[3][1] = -swy
  t[3][2] = swx*cwy
  t[3][3] = cwx*cwy
  t[3][4] = p[3]

  return setmetatable(t, mt)
end

local function mul(t1, t2)
  local t = {}
  -- Matrix * Vector
  if type(t2[1]) == "number" then
    for i = 1,4 do
      t[i] = t1[i][1] * t2[1]
      + t1[i][2] * t2[2]
      + t1[i][3] * t2[3]
      + t1[i][4] * t2[4]
    end
    return vector.new(t)
    -- Matrix * Matrix
  elseif type(t2[1]) == "table" then
    for i = 1,4 do
      t[i] = {}
      for j = 1,4 do
        t[i][j] = t1[i][1] * t2[1][j]
        + t1[i][2] * t2[2][j]
        + t1[i][3] * t2[3][j]
        + t1[i][4] * t2[4][j]
      end
    end
    return setmetatable(t, mt)
  end
end

-- Use the 6D vector
local function tostring(t, formatstr)
  return tostring( Transform.position6D( t ) )
end

mt.__mul = mul
mt.__tostring = tostring

return Transform