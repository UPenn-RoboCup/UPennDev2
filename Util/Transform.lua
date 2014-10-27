local vector = require'vector'
local quaternion = require'quaternion'
local Transform = {}

local cos = math.cos
local sin = math.sin

local mt = {}

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
  local ca = cos(a)
  local sa = sin(a)
  local t = {}
  t[1] = vector.new({ca, -sa, 0, 0})
  t[2] = vector.new({sa, ca, 0, 0})
  t[3] = vector.new({0, 0, 1, 0})
  t[4] = vector.new({0, 0, 0, 1})
  return setmetatable(t, mt)
end

function Transform.rotY(a)
  local ca = cos(a)
  local sa = sin(a)
  local t = {}
  t[1] = vector.new({ca, 0, sa, 0})
  t[2] = vector.new({0, 1, 0, 0})
  t[3] = vector.new({-sa, 0, ca, 0})
  t[4] = vector.new({0, 0, 0, 1})
  return setmetatable(t, mt)
end

function Transform.rotX(a)
  local ca = cos(a)
  local sa = sin(a)
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

-- Recovering Euler Angles
-- Good resource: http://www.vectoralgebra.info/eulermatrix.html
function Transform.to_zyz(t)
  -- Modelling and Control of Robot Manipulators, pg. 30
  -- Lorenzo Sciavicco and Bruno Siciliano
  local e = vector.zeros(3)
  e[1]=math.atan2(t[2][3],t[1][3]) -- Z (phi)
  e[2]=math.atan2(math.sqrt( t[1][3]^2 + t[2][3]^2),t[3][3]) -- Y (theta)
  e[3]=math.atan2(t[3][2],-t[3][1]) -- Z' (psi)
  return e
end

-- RPY is XYZ convention
function Transform.to_rpy(t)
  -- http://planning.cs.uiuc.edu/node103.html
  -- returns [roll, pitch, yaw] vector
  local e = vector.zeros(3)
  e[1]=math.atan2(t[3][2],t[3][3]) --Roll
  e[2]=math.atan2(-t[3][1],math.sqrt( t[3][2]^2 + t[3][3]^2) ) -- Pitch
  e[3]=math.atan2(t[2][1],t[1][1]) -- Yaw
  return e
end

function Transform.position6D(tr)
  return vector.new{
  tr[1][4],tr[2][4],tr[3][4],
  math.atan2(tr[3][2],tr[3][3]),
  -math.asin(tr[3][1]),
  math.atan2(tr[2][1],tr[1][1])
  }
end

-- Rotation Matrix to quaternion
-- from Yida.  Adapted to take a transformation matrix
Transform.to_quaternion = function( t )
  local offset = vector.new{t[1][4],t[2][4],t[3][4]}
  local q = quaternion.new()
  local tr = t[1][1] + t[2][2] + t[3][3]
  if tr > 0 then
    local S = math.sqrt(tr + 1.0) * 2
    q[1] = 0.25 * S
    q[2] = (t[3][2] - t[2][3]) / S
    q[3] = (t[1][3] - t[3][1]) / S
    q[4] = (t[2][1] - t[1][2]) / S
  elseif t[1][1] > t[2][2] and t[1][1] > t[3][3] then
    local S = math.sqrt(1.0 + t[1][1] - t[2][2] - t[3][3]) * 2
    q[1] = (t[3][2] - t[2][3]) / S
    q[2] = 0.25 * S
    q[3] = (t[1][2] + t[2][1]) / S 
    q[4] = (t[1][3] + t[3][1]) / S
  elseif t[2][2] > t[3][3] then
    local S = math.sqrt(1.0 + t[2][2] - t[1][1] - t[3][3]) * 2
    q[1] = (t[1][3] - t[3][1]) / S
    q[2] = (t[1][2] + t[2][1]) / S 
    q[3] = 0.25 * S
    q[4] = (t[2][3] + t[3][2]) / S
  else
    local S = math.sqrt(1.0 + t[3][3] - t[1][1] - t[2][2]) * 2
    q[1] = (t[2][1] - t[1][2]) / S
    q[2] = (t[1][3] + t[3][1]) / S 
    q[3] = (t[2][3] + t[3][2]) / S
    q[4] = 0.25 * S
  end
  return q, offset
end

-- Can give the position
Transform.from_quaternion = function(q,pos)
  local t = Transform.eye()
  t[1][1] = 1 - 2 * q[3] * q[3] - 2 * q[4] * q[4]
  t[1][2] = 2 * q[2] * q[3] - 2 * q[4] * q[1]
  t[1][3] = 2 * q[2] * q[4] + 2 * q[3] * q[1]
  t[2][1] = 2 * q[2] * q[3] + 2 * q[4] * q[1]
  t[2][2] = 1 - 2 * q[2] * q[2] - 2 * q[4] * q[4]
  t[2][3] = 2 * q[3] * q[4] - 2 * q[2] * q[1]
  t[3][1] = 2 * q[2] * q[4] - 2 * q[3] * q[1]
  t[3][2] = 2 * q[3] * q[4] + 2 * q[2] * q[1]
  t[3][3] = 1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]
  if pos then return Transform.trans(unpack(pos))*t end
  return t
end

function Transform.transform6D(p)
  local t = {}

  local cwx = cos(p[4])
  local swx = sin(p[4])
  local cwy = cos(p[5])
  local swy = sin(p[5])
  local cwz = cos(p[6])
  local swz = sin(p[6])

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
  if type(t2[1]) == "number" then
    -- Matrix * Vector
    for i = 1,4 do
      t[i] = t1[i][1] * t2[1]
      + t1[i][2] * t2[2]
      + t1[i][3] * t2[3]
      + t1[i][4] * t2[4]
    end
    return vector.new(t)
  elseif type(t2[1]) == "table" then
    -- Matrix * Matrix
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

-- Copy
function Transform.copy(tt)
  if type(tt)=='table' then
    -- Copy the table
    local t = {}
    t[1] = vector.copy(tt[1])
    t[2] = vector.copy(tt[2])
    t[3] = vector.copy(tt[3])
    t[4] = vector.copy(tt[4])
    return setmetatable(t, mt)
  end
  local t = Transform.eye()
  for i=1,3 do
    for j=1,4 do
      t[i][j] = tt[i][j]
    end
  end
  -- copy a tensor
  return t
end

-- Do it unsafe; assume a table
function Transform.new(tt)
  vector.new(tt[1])
  vector.new(tt[2])
  vector.new(tt[3])
  vector.new(tt[4])
  return setmetatable(tt, mt)
end

-- Use the 6D vector
local function tostring(t, formatstr)
  return tostring( Transform.position6D(t), formatstr )
end
-- Full matrix for the library tostring helper
Transform.tostring = function(tr)
  local pr = {}
  for i=1,4 do
    local row = {}
    for j=1,4 do
      table.insert(row,string.format('%6.3f',tr[i][j]))
    end
    local c = table.concat(row,', ')
    table.insert(pr,string.format('[%s]',c))
  end
  return table.concat(pr,'\n')
end

mt.__mul = mul
mt.__tostring = tostring
--mt.__call = function(self,idx) print('idx',idx,type(idx),self[idx]); return self[idx] end

return Transform
