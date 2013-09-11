-- Transformation Matrix functions
local torch = require'torch'
torch.Tensor = torch.DoubleTensor
local quaternion = require'quaternion'

-- TODO: Is this actually a good name?
local libTransform = {}

-- TODO: Make sure the helper functions are working properly!
libTransform.rotX = function(t)
  -- Homogeneous transformation representing a rotation of theta
  -- about the X axis.
  local ct = math.cos(t)
  local st = math.sin(t)
  local r = torch.eye(4)
  r[2][2] = ct
  r[3][3] = ct
  r[2][3] = -st
  r[3][2] = st
  return r
end

libTransform.rotY = function(t)
  -- Homogeneous transformation representing a rotation of theta
  -- about the Y axis.
  local ct = math.cos(t)
  local st = math.sin(t)
  local r = torch.eye(4)
  r[1][1] = ct
  r[3][3] = ct
  r[1][3] = st
  r[3][1] = -st
  return r
end

libTransform.rotZ = function(t)
  -- Homogeneous transformation representing a rotation of theta
  -- about the Z axis.
  local ct = math.cos(t)
  local st = math.sin(t)
  local r = torch.eye(4)
  r[1][1] = ct
  r[2][2] = ct
  r[1][2] = -st
  r[2][1] = st
  return r
end

libTransform.trans = function(dx, dy, dz)
  local t = torch.eye(4)
  t[1][4] = dx
  t[2][4] = dy
  t[3][4] = dz
  return t
end

-- Rotation matrix to Roll Pitch Yaw
-- Yida's from http://planning.cs.uiuc.edu/node102.html
libTransform.to_rpy = function( R )
  local y = math.atan2(R[2][1], R[1][1])
  local p = math.atan2(-R[3][1], math.sqrt(R[3][2]^2+R[3][3]^2))
  local r = math.atan2(R[3][2], R[3][3])
  return torch.Tensor{r,p,y}
end

-- This gives xyz,rpy, 
-- so should be better than the to_rpy function...
function libTransform.position6D(tr)
  return torch.Tensor{
  tr[1][4],tr[2][4],tr[3][4],
  math.atan2(tr[3][2],tr[3][3]),
  -math.asin(tr[3][1]),
  math.atan2(tr[2][1],tr[1][1])
  }
end

-- From Yida, with a resourse
-- http://planning.cs.uiuc.edu/node102.html
libTransform.from_rpy = function( rpy )
  local alpha = rpy[3]
  local beta  = rpy[2]
  local gamma = rpy[1]
  local t = torch.eye(4)
  t[1][1] = math.cos(alpha)
  t[1][1] = math.cos(alpha) * math.cos(beta)
  t[2][1] = math.sin(alpha) * math.cos(beta)
  t[3][1] = -math.sin(beta)
  t[1][2] = math.cos(alpha) * math.sin(beta) * math.sin(gamma) - math.sin(alpha) * math.cos(gamma)
  t[2][2] = math.sin(alpha) * math.sin(beta) * math.sin(gamma) + math.cos(alpha) * math.cos(gamma)
  t[3][2] = math.cos(beta) * math.sin(gamma)
  t[1][3] = math.cos(alpha) * math.sin(beta) * math.cos(gamma) + math.sin(alpha) * math.sin(gamma)
  t[2][3] = math.sin(alpha) * math.sin(beta) * math.cos(gamma) - math.cos(alpha) * math.sin(gamma)
  t[3][3] = math.cos(beta) * math.cos(gamma)
  return t
end

-- Rotation Matrix to quaternion
-- from Yida.  Adapted to take a transformation matrix
libTransform.to_quaternion = function(t)
  local offset = torch.Tensor{t[1][4],t[2][4],t[3][4]}
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

function libTransform.from_quaternion( q, root )
  local t = torch.eye(4)
  t[1][1] = 1 - 2 * q[3] * q[3] - 2 * q[4] * q[4]
  t[1][2] = 2 * q[2] * q[3] - 2 * q[4] * q[1]
  t[1][3] = 2 * q[2] * q[4] + 2 * q[3] * q[1]
  t[2][1] = 2 * q[2] * q[3] + 2 * q[4] * q[1]
  t[2][2] = 1 - 2 * q[2] * q[2] - 2 * q[4] * q[4]
  t[2][3] = 2 * q[3] * q[4] - 2 * q[2] * q[1]
  t[3][1] = 2 * q[2] * q[4] - 2 * q[3] * q[1]
  t[3][2] = 2 * q[3] * q[4] + 2 * q[2] * q[1]
  t[3][3] = 1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]
  if root then return libTransform.trans(unpack(root))*t end
  return t
end

function libTransform.to_angle_axis( tr )
  local axis = torch.Tensor(3)
  axis[1] = tr[3][2]-tr[2][3]
  axis[2] = tr[1][3]-tr[3][1]
  axis[3] = tr[2][1]-tr[1][2]
  local r = torch.norm(axis)
  local t = tr[1][1]+tr[2][2]+tr[3][3]
  local angle = math.atan2(r,t-1)
  return angle, axis
end

-- http://en.wikipedia.org/wiki/Rotation_matrix#Axis_and_angle
function libTransform.from_angle_axis( angle, axis )
  axis:div(axis:norm())
  local x = axis[1]
  local y = axis[2]
  local z = axis[3]
  local s = math.sin(angle)
  local c = math.cos(angle)
  local nc = 1-c
  local t = torch.eye(4)
  t[1][1] = x*x*nc+c
  t[1][2] = x*y*nc-z*s
  t[1][3] = x*z*nc+y*s
  t[2][1] = y*x*nc+z*s
  t[2][2] = y*y*nc+c
  t[2][3] = y*z*nc-x*s
  t[3][1] = z*x*nc-y*s
  t[3][2] = z*y*nc+x*s
  t[3][3] = z*z*nc+c
  return t
end

-- Assume dipole and root are torch objects...
function libTransform.from_dipole( dipole, root )
  local z_axis = torch.Tensor{0,0,1}
  local axis   = torch.cross(z_axis,dipole)
  local angle  = math.acos(z_axis:dot(dipole))
  local r = libTransform.from_angle_axis( angle, axis )
  if root then return libTransform.trans(unpack(root))*r end
  return r
end

-- from 6d x,y,z,r,p,y
function libTransform.transform6D(p)

  local cwx = math.cos(p[4])
  local swx = math.sin(p[4])
  local cwy = math.cos(p[5])
  local swy = math.sin(p[5])
  local cwz = math.cos(p[6])
  local swz = math.sin(p[6])

  local t = torch.eye(4)

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

  return t
end

-- Quicker inverse, since Transformation matrices are special cases
function libTransform.inv(a)
  local t = torch.eye(4)
  -- Rotation component transposed
  local r_t = a:sub(1,3,1,3):t()
  t:sub(1,3,1,3):copy(r_t)
  -- Translation portion
  local p   = a:select(2,4):narrow(1,1,3)
  local t_p = t:select(2,4):narrow(1,1,3)
  t_p:mv(r_t,p):mul(-1)
  return t
end

--[[
  -- These should form an orthonormal basis
  -- Each norm is 1
  t2n = t2n / vector.norm(t2n);
  s2s = s2s / vector.norm(s2s);
  --Check the properties:
  --if( t2n*s2s > 0.05 ) then
  --print('Dot product should be zero: ', n2t*sL2sR);
  --end
  -- Find the cross product
  chest = cross(s2s,t2n);

  -- Remap the coordinates
  u = vector.new({chest[3],chest[1],chest[2]})
  v = vector.new({s2s[3],s2s[1],s2s[2]})
  w = vector.new({t2n[3],t2n[1],t2n[2]})

  -- Find the closest Orthonormal matrix
  local matrix = require 'matrix'
  M = matrix.transpose( matrix{u,v,w} )
  -- Real Rotation Matrix
  R = M*(matrix.transpose(M)*M)^-1/2
--]]

return libTransform