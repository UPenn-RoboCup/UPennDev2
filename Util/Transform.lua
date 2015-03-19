local Transform = {}
local mt = {}

local vector = require'vector'
local quaternion = require'quaternion'

local cos = math.cos
local sin = math.sin
local atan2 = math.atan2
local sqrt = math.sqrt
local vnew, vcopy = vector.new, vector.copy

function Transform.inv(a)
	local p = {a[1][4],a[2][4],a[3][4]}
	local r = {
	  {a[1][1],a[2][1],a[3][1]},
	  {a[1][2],a[2][2],a[3][2]},
	  {a[1][3],a[2][3],a[3][3]}
	}
	return setmetatable({
		{r[1][1], r[1][2], r[1][3], -(r[1][1]*p[1]+r[1][2]*p[2]+r[1][3]*p[3])},
		{r[2][1], r[2][2], r[2][3], -(r[2][1]*p[1]+r[2][2]*p[2]+r[2][3]*p[3])},
		{r[3][1], r[3][2], r[3][3], -(r[3][1]*p[1]+r[3][2]*p[2]+r[3][3]*p[3])},
		{0,0,0,1}
	}, mt)
end

local function eye()
  return setmetatable({
		{1, 0, 0, 0},
		{0, 1, 0, 0},
		{0, 0, 1, 0},
		{0, 0, 0, 1}
	}, mt)	
end
Transform.eye = eye

function Transform.rotZ(a)
  local ca = cos(a)
  local sa = sin(a)
  return setmetatable({
	  {ca, -sa, 0, 0},
	  {sa, ca, 0, 0},
	  {0, 0, 1, 0},
	  {0, 0, 0, 1}
	}, mt)
end

function Transform.rotY(a)
  local ca = cos(a)
  local sa = sin(a)
  return setmetatable({
	  {ca, 0, sa, 0},
	  {0, 1, 0, 0},
	  {-sa, 0, ca, 0},
	  {0, 0, 0, 1}
	}, mt)	
end

function Transform.rotX(a)
  local ca = cos(a)
  local sa = sin(a)
  return setmetatable({
	  {1, 0, 0, 0},
	  {0, ca, -sa, 0},
	  {0, sa, ca, 0},
	  {0, 0, 0, 1}
	}, mt)
end

function Transform.trans(dx, dy, dz)
  return setmetatable({
	  {1, 0, 0, dx},
	  {0, 1, 0, dy},
	  {0, 0, 1, dz},
	  {0, 0, 0, 1}
	}, mt)
end

-- Recovering Euler Angles
-- Good resource: http://www.vectoralgebra.info/eulermatrix.html
function Transform.to_zyz(t)
  -- Modelling and Control of Robot Manipulators, pg. 30
  -- Lorenzo Sciavicco and Bruno Siciliano
  local e = vector.zeros(3)
  e[1]=atan2(t[2][3],t[1][3]) -- Z (phi)
  e[2]=atan2(sqrt( t[1][3]^2 + t[2][3]^2),t[3][3]) -- Y (theta)
  e[3]=atan2(t[3][2],-t[3][1]) -- Z' (psi)
  return e
end

-- RPY is XYZ convention
function Transform.to_rpy(t)
  -- http://planning.cs.uiuc.edu/node103.html
  -- returns [roll, pitch, yaw] vector
  local e = vector.zeros(3)
  e[1]=atan2(t[3][2],t[3][3]) --Roll
  e[2]=atan2(-t[3][1],sqrt( t[3][2]^2 + t[3][3]^2) ) -- Pitch
  e[3]=atan2(t[2][1],t[1][1]) -- Yaw
  return e
end

function Transform.position6D(tr)
  return vnew{
  tr[1][4],tr[2][4],tr[3][4],
  atan2(tr[3][2],tr[3][3]),
  -math.asin(tr[3][1]),
  atan2(tr[2][1],tr[1][1])
  }
end

function Transform.position(tr)
  return vnew{tr[1][4],tr[2][4],tr[3][4]}
end

function Transform.position4(tr)
  return vnew{tr[1][4],tr[2][4],tr[3][4],tr[4][4]}
end

-- Rotation Matrix to quaternion
-- from Yida.  Adapted to take a transformation matrix
function Transform.to_quaternion( t )
  local q = quaternion.new()
  local tr = t[1][1] + t[2][2] + t[3][3]
  if tr > 0 then
    local S = sqrt(tr + 1.0) * 2
    q[1] = 0.25 * S
    q[2] = (t[3][2] - t[2][3]) / S
    q[3] = (t[1][3] - t[3][1]) / S
    q[4] = (t[2][1] - t[1][2]) / S
  elseif t[1][1] > t[2][2] and t[1][1] > t[3][3] then
    local S = sqrt(1.0 + t[1][1] - t[2][2] - t[3][3]) * 2
    q[1] = (t[3][2] - t[2][3]) / S
    q[2] = 0.25 * S
    q[3] = (t[1][2] + t[2][1]) / S 
    q[4] = (t[1][3] + t[3][1]) / S
  elseif t[2][2] > t[3][3] then
    local S = sqrt(1.0 + t[2][2] - t[1][1] - t[3][3]) * 2
    q[1] = (t[1][3] - t[3][1]) / S
    q[2] = (t[1][2] + t[2][1]) / S 
    q[3] = 0.25 * S
    q[4] = (t[2][3] + t[3][2]) / S
  else
    local S = sqrt(1.0 + t[3][3] - t[1][1] - t[2][2]) * 2
    q[1] = (t[2][1] - t[1][2]) / S
    q[2] = (t[1][3] + t[3][1]) / S 
    q[3] = (t[2][3] + t[3][2]) / S
    q[4] = 0.25 * S
  end
  return q, vnew{t[1][4],t[2][4],t[3][4]}
end

-- Can give the position
function Transform.from_quaternion(q, pos)
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

function Transform.transform6D(p) --point address
  local cwx = cos(p[4]) --4,5,6 : YPR angles
  local swx = sin(p[4])
  local cwy = cos(p[5])
  local swy = sin(p[5])
  local cwz = cos(p[6])
  local swz = sin(p[6])

	local t = eye()

  t[1][1] = cwy*cwz
  t[1][2] = swx*swy*cwz-cwx*swz
  t[1][3] = cwx*swy*cwz+swx*swz
  t[1][4] = p[1]
	--
  t[2][1] = cwy*swz
  t[2][2] = swx*swy*swz+cwx*cwz
  t[2][3] = cwx*swy*swz-swx*cwz
  t[2][4] = p[2]
	--
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
      + t1[i][4] * (t2[4] or 1)
    end
    return vnew(t)
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
    return setmetatable({
    	vector.copy(tt[1]),
    	vector.copy(tt[2]),
    	vector.copy(tt[3]),
    	vector.copy(tt[4])
		}, mt)
  end
  local t = eye()
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
	--[[
  vnew(tt[1])
  vnew(tt[2])
  vnew(tt[3])
  vnew(tt[4])
	--]]
  return setmetatable(tt, mt)
end

-- Use the 6D vector
local function tostring(t, formatstr)
  return tostring( Transform.position6D(t), formatstr )
end
-- Full matrix for the library tostring helper
function Transform.tostring(tr)
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
mt.__tostring = Transform.tostring --tostring
--mt.__call = function(self,idx) print('idx',idx,type(idx),self[idx]); return self[idx] end

return Transform
