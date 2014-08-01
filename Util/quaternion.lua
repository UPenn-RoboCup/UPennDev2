-----------------------------------------------------------------
-- Quaternion
-- Performs operations on quaternions
-- (c) Stephen McGill, Yida Zhang 2013
---------------------------------

local vector=require'vector'
local util=require'util'
local quaternion = {}
local mt = {}

local function cross(v1, v2)
  return vector.new{
  ( (v1[2] * v2[3]) - (v1[3] * v2[2]) ),
  - ( (v1[1] * v2[3]) - (v1[3] * v2[1]) ),
  ( (v1[1] * v2[2]) - (v1[2] * v2[1]) ),
	}
end

-- New quaternion from a 3 or 4 element table
function quaternion.new(t,alpha)
  if not t then return setmetatable({1,0,0,0}, mt) end
  if #t==4 then return setmetatable(t, mt) end
  -- Rotation vector to a quaternion
  assert(#t==3,'Must give a 3 or 4 element table to make a quaternion')
  -- Grab the norm of the rotation vector
  local wNorm = vector.norm(t)
  -- Avoid small norms
  if wNorm < 1e-6 then return setmetatable({1,0,0,0}, mt) end
  -- If given, use alpha as the amount of rotation about the axis vector t
  alpha = alpha or wNorm
  local scale = math.sin(alpha/2) / wNorm
	local q = {
  	math.cos(alpha/2),
  	scale*t[1], scale*t[2], scale*t[3],
	}
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
	local q_c = {
		q[1], -1*q[2], -1*q[3], -1*q[4],
	}
  return setmetatable(q_c, mt)
end

-- Make a unit quaternion
-- Assumes that q is already a quaternion
function quaternion.unit( v1 )
  local wNorm = vector.norm(v1)
  if wNorm < 1e-6 then return quaternion.new() end
  return setmetatable(
		{
			v1[1]/wNorm,
			v1[2]/wNorm,
			v1[3]/wNorm,
			v1[4]/wNorm
		},
		mt)
end

-- Return a rotation vector
-- From Yida
function quaternion.vector(q)
  assert( math.abs(q[1])<=1, 'Bad unit quaternion' )
  local alphaW = 2*math.acos(q[1])
	-- Avoid the divide by zero scenario
  if alphaW < 1e-6 then return vector.zeros(3) end
	local factor = alphaW / math.sin(alphaW/2)
  return factor * vector.new({q[2],q[3],q[4]})
end

function quaternion.from_angle_axis(angle,axis)
  local norm = vector.norm(axis)
  if norm<1e-6 then return quaternion.new() end
  local s = math.sin(angle/2)/norm
  return quaternion.new({
    math.cos(angle/2),
    s*axis[1],
    s*axis[2],
    s*axis[3],
  })
end

-- For the clicking of the object to pickup
-- dipole must be normalized, first
function quaternion.from_dipole( dipole )
  local z_axis = vector.new{0,0,1}
--  local axis   = cross(dipole,z_axis)
  local axis   = cross(z_axis,dipole)
  local angle  = math.acos(dipole * z_axis)
  return quaternion.from_angle_axis(angle,axis)
end

function quaternion.angle_axis(q)
  q = quaternion.unit( q )
  assert( math.abs(q[1])<=1, 'Bad unit quaternion' )
  local angle = 2*math.acos(q[1])
  -- Avoid the divide by zero scenario
	if angle< 1e-6 then return 0, vector.new({1,0,0}) end
	return angle, vector.new({q[2],q[3],q[4]})/math.sin(angle/2)
end

-- Get the angle between two quaternions
local function diff(q0,q1)
	local angle0, axis0 = quaternion.angle_axis(
		quaternion.conjugate(q0) * q1
	)
	local angle1, axis1 = quaternion.angle_axis(
		quaternion.conjugate(q0) * -q1
	)
	--[[
	print('==')
	print('A: ',angle0*RAD_TO_DEG, axis0)
	print('B: ',angle1*RAD_TO_DEG, axis1)
	print('--')
	--]]
	-- Check the other direction
	if math.abs(angle1)<math.abs(angle0) then
		return angle1, axis1
	end
	return angle0, axis0
end
quaternion.diff = diff

-- https://en.wikipedia.org/wiki/Slerp
function quaternion.slerp(q0,q1,t)
	t = t or .5
	----[[
	local q0_prime = quaternion.conjugate(q0)
	local prodQuat = q0_prime * q1
	local angle, axis = quaternion.angle_axis(prodQuat)
	--]]
	--[[
	local angle, axis = diff(q0,q1)
	--]]
	local quadT = quaternion.from_angle_axis(angle * t,axis)
	local qSlerp = q0 * quadT
	return qSlerp
end

-- https://theory.org/software/qfa/writeup/node12.html
-- s0, s1: inner quadrangle points
function quaternion.squad(q0,q1,t,s0,s1)
	local sl = quaternion.slerp
	return sl(
		sl(q0,q1,t),
		sl(s0,s1,t),
		2*t*(1-t)
	)
end

-- Metatable methods are local
local function add(q1, q2)
  local q = {}
  for i,v in ipairs(q1) do q[i] = v + q2[i] end
  return setmetatable(q, mt)
end

--http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/
local function negate(q)
	return setmetatable({
		-q[1],-q[2],-q[3],-q[4]
	},mt)
end

local function mul(q1, q2)
  local a1,b1,c1,d1 = unpack(q1)
  local a2,b2,c2,d2 = unpack(q2)
  local q = {
  a1*a2 - b1*b2 - c1*c2 - d1*d2,
  a1*b2 + b1*a2 + c1*d2 - d1*c2,
  a1*c2 - b1*d2 + c1*a2 + d1*b2,
  a1*d2 + b1*c2 - c1*b2 + d1*a2
	}
  return quaternion.unit( q )
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
mt.__sub = diff
mt.__mul = mul
mt.__unm = negate
mt.__tostring = tostring

return quaternion


-- Take the average of a set of quaternions
-- TODO: Remove the torch dependence
-- NOTE: without torch, this may be slow
--[[
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
--]]
