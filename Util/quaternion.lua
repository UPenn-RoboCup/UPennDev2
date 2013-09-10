-----------------------------------------------------------------
-- Quaternion
-- Performs operations on quaternions
-- (c) Stephen McGill, Yida Zhang 2013
---------------------------------

local vector=require'vector'
local util=require'util'
local quaternion = {}

local mt = {}

-- New quaternion from a 3 or 4 element table
function quaternion.new(t,alpha)
  if not t then return setmetatable({1,0,0,0}, mt) end
  if #t==4 then return setmetatable(t, mt) end
  -- Rotation vector to a quaternion
  assert(#t==3,'Must give a 3 or 4 element table to make a quaternion')
  local q = {1,0,0,0}
  -- Grab the norm of the rotation vector
  local wNorm = vector.norm(t)
  -- Avoid small norms
  if wNorm < 1e-6 then return setmetatable(q, mt) end
  -- If given, use alpha as the amount of rotation about the axis vector t
  alpha = alpha or wNorm
  local scale = math.sin(alpha/2) / wNorm
  q[1] = math.cos(alpha/2)
  q[2] = scale*t[1]
  q[3] = scale*t[2]
  q[4] = scale*t[3]
  return setmetatable(q, mt)
end

-- Return the Roll/Pitch/Yaw of this quaternion
-- Modified from Yida's UKF
-- http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
function quaternion.to_rpy( q )
  local rpy = {}
  rpy[1] = util.mod_angle( 
    math.atan2( 2*(q[1]*q[2]+q[3]*q[4]), 1-2*(q[2]*q[2]+q[3]*q[3]) )
  )
  rpy[2] = util.mod_angle(
    math.asin( util.procFunc(2*(q[1]*q[3]-q[4]*q[2]),0,1) )
  )
  rpy[3] = util.mod_angle(
    math.atan2(2*(q[1]*q[4]+q[2]*q[3]), 1-2*(q[3]*q[3]+q[4]*q[4]))
  )
  return vector.new(rpy)
end

function quaternion.conjugate( q )
  q[2] = -1*q[2]
  q[3] = -1*q[3]
  q[4] = -1*q[4]
  return setmetatable(q, mt)
end

-- Make a unit quaternion
-- Assumes that q is already a quaternion
function quaternion.unit( v1 )
  local v = {1,0,0,0}
  local wNorm = vector.norm(v1)
  if wNorm < 1e-6 then return setmetatable(v, mt) end
  for i = 1, #v1 do v[i] = v1[i] / wNorm end
  return setmetatable(v, mt)
end

-- Return a rotation vector
-- From Yida
function quaternion.vector(q)
  assert( math.abs(q[1])<=1, 'Bad unit quaternion' )
  local alphaW = 2*math.acos(q[1])
  local v = vector.new({0,0,0})
  -- Avoid the divide by zero scenario
  if alphaW > 1e-6 then
    v[1] = q[2] / math.sin(alphaW/2) * alphaW
    v[2] = q[3] / math.sin(alphaW/2) * alphaW
    v[3] = q[4] / math.sin(alphaW/2) * alphaW
  end
  return v
end

function quaternion.from_angle_axis(angle,axis)
  local s = math.sin(angle/2)
  return quaternion.new({
    math.cos(angle/2),
    s*axis[1],
    s*axis[2],
    s*axis[3],
  })
end

-- For the clicking of the object to pickup
function quaternion.from_dipole( dipole )
  local z_axis = vector.new{0,0,1}
  local axis = vector.cross(dipole,z_axis)
  local dot = (dipole/vector.norm(dipole))*z_axis
  -- Forward-inverse identities
  -- http://mathworld.wolfram.com/InverseTrigonometricFunctions.html
  local fii = math.sqrt(1-dot^2)
  return quaternion.new{
    dot,
    fii*axis[1],
    fii*axis[2],
    fii*axis[3]
  }
end

function quaternion.angle_axis(q)
  q = quaternion.unit( q )
  assert( math.abs(q[1])<=1, 'Bad unit quaternion' )
  local angle = 2*math.acos(q[1])
  local v = vector.new({1,0,0})
  -- Avoid the divide by zero scenario
  if angle > 1e-6 then
    v[1] = q[2] / math.sin(angle/2)
    v[2] = q[3] / math.sin(angle/2)
    v[3] = q[4] / math.sin(angle/2)
  end
  return util.mod_angle(angle), v
end

-- Take the average of a set of quaternions
-- TODO: Remove the torch dependence
-- NOTE: without torch, this may be slow
function quaternion.mean( QMax, qInit )
  local torch = require'torch'
  local qIter = torch.DoubleTensor(4):copy(qInit)
  local e = torch.DoubleTensor(3, QMax:size(2)):fill(0)
  local iter = 0
  local diff = 0
  repeat
    iter = iter + 1
    for i = 1, QMax:size(2) do
      local ei = e:narrow(2, i, 1):fill(0)
      local qi = QMax:narrow(2, i, 1)
      local eQ = QuatMul(qi, QuatInv(qIter))
      ei:copy(Quat2Vector(eQ))
    end
    local eMean = quaternion.new( torch.mean(e,2) )
    
    local qIterNext = eMean * qIter
    -- Compare the roation vectors
    local diff = vector.norm(quaternion.vector(qIterNext)-quaternion.vector(e2))
    qIter = qIterNext
  until diff < 1e-6
  return qIter, e
end

-- Metatable methods are local
local function add(q1, q2)
  local q = {}
  for i,v in ipairs(q1) do q[i] = v + q2[i] end
  return setmetatable(q, mt)
end

local function mul(q1, q2)
  local a1,b1,c1,d1 = unpack(q1)
  local a2,b2,c2,d2 = unpack(q2)
  local q = {}
  q[1] = a1*a2 - b1*b2 - c1*c2 - d1*d2
  q[2] = a1*b2 + b1*a2 + c1*d2 - d1*c2
  q[3] = a1*c2 - b1*d2 + c1*a2 + d1*b2
  q[4] = a1*d2 + b1*c2 - c1*b2 + d1*a2
  return setmetatable(q, mt)
end

-- Same tostring as a vector
local function tostring(v1, formatstr)
  formatstr = formatstr or "%g"
  local str = "{"..string.format(formatstr, v1[1])
  for i = 2, #v1 do
    str = str..", "..string.format(formatstr,v1[i])
  end
  str = str.."}"
  return str
end

-- Set the metatable values
mt.__add = add
mt.__mul = mul
mt.__tostring = tostring

return quaternion