-- Transformation Matrix functions
local torch = require'torch'
local vector = require'vector'
local quaternion = require'quaternion'
local cos = math.cos
local sin = math.sin

-- TODO: Is this actually a good name?
local libTransform = {}

libTransform.eye = function()
  return torch.eye(4)
end

-- TODO: Make sure the helper functions are working properly!
libTransform.rotX = function(t)
  -- Homogeneous transformation representing a rotation of theta
  -- about the X axis.
  local ct = cos(t)
  local st = sin(t)
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
  local ct = cos(t)
  local st = sin(t)
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
  local ct = cos(t)
  local st = sin(t)
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

-- Recovering Euler Angles
-- Good resource: http://www.vectoralgebra.info/eulermatrix.html
libTransform.to_zyz = function(t)
  -- Modelling and Control of Robot Manipulators, pg. 30
  -- Lorenzo Sciavicco and Bruno Siciliano
	local z, y, zz
	local s = math.abs(t[3][3])
	if s>0.99 then
		z = 0
		y = (t[3][3]>0 and 0) or math.pi
		zz = math.asin( t[2][1] )
	else
		z = math.atan2(t[2][3],t[1][3]) -- Z (phi)
		y = math.atan2(math.sqrt( t[1][3]^2 + t[2][3]^2),t[3][3]) -- Y (theta)
		zz = math.atan2(t[3][2],-t[3][1]) -- Z' (psi)
		s = false
	end
  return torch.Tensor{z,y,zz}, s
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
  t[1][1] = cos(alpha)
  t[1][1] = cos(alpha) * cos(beta)
  t[2][1] = sin(alpha) * cos(beta)
  t[3][1] = -sin(beta)
  t[1][2] = cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma)
  t[2][2] = sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma)
  t[3][2] = cos(beta) * sin(gamma)
  t[1][3] = cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma)
  t[2][3] = sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma)
  t[3][3] = cos(beta) * cos(gamma)
  return t
end

-- Rotation Matrix to quaternion
-- from Yida.  Adapted to take a transformation matrix
libTransform.to_quaternion = function(t)
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
  return q, {t[1][4],t[2][4],t[3][4]}
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
  --if root then return libTransform.trans(unpack(root))*t end
	if root then
		t[1][4] = root[1]
		t[2][4] = root[2]
		t[3][4] = root[3]
	end
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

-- Take in two vectors (9 dim and 3 dim)
-- Mostly for rcm
function libTransform.from_flat( trans, rot )
	local t = libTransform.trans(unpack(trans))
	if not rot then return t end
	for i,v in ipairs(rot) do
		local ii = ((i-1) % 3) + 1
		local jj = ((i-ii) / 3) + 1
		t[ii][jj] = v
	end
	return t
end
function libTransform.flatten( tr )
	-- TODO
end

-- http://en.wikipedia.org/wiki/Rotation_matrix#Axis_and_angle
function libTransform.from_angle_axis( angle, axis )
  axis:div(axis:norm())
  local x = axis[1]
  local y = axis[2]
  local z = axis[3]
  local s = sin(angle)
  local c = cos(angle)
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

  local cwx = cos(p[4])
  local swx = sin(p[4])
  local cwy = cos(p[5])
  local swy = sin(p[5])
  local cwz = cos(p[6])
  local swz = sin(p[6])

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

function libTransform.position(tr)
  return vector.new(tr:select(2,4):narrow(1,1,3))
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




function libTransform.trans6D(p, xyz)
--enter 6D transform and apply translation by xyz
--and return 6D transform


  local cwx = cos(p[4])
  local swx = sin(p[4])
  local cwy = cos(p[5])
  local swy = sin(p[5])
  local cwz = cos(p[6])
  local swz = sin(p[6])

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



  local xoffset = t[1][1]*xyz[1]+t[1][2]*xyz[2]+t[1][3]*xyz[3]
  local yoffset = t[2][1]*xyz[1]+t[2][2]*xyz[2]+t[2][3]*xyz[3]
  local zoffset = t[3][1]*xyz[1]+t[3][2]*xyz[2]+t[3][3]*xyz[3]

  return{p[1]+xoffset,p[2]+yoffset,p[3]+zoffset,p[4],p[5],p[6]}
end



-- Find the closest Orthonormal matrix for the rotaiton component
-- Calculating square roots via LAPACK:
-- http://math.stackexchange.com/questions/106774/matrix-square-root
--[[
function libTransform.nearest( rotation )
  local tr = torch.eye(4)
  local rot = tr:sub(1,3,1,3)
  local tmp = torch.mm(rotation:t(),rotation)
  local tmp2 = tmp
  rot:mm( rotation, (tmp^-1/2) )
end
--]]

-- Put element of t into tr
libTransform.copy = function(t)
  if type(t)=='table' then
    -- Copy the table
    return torch.Tensor(t)
  end
  -- copy a tensor
  return t:clone()
end

libTransform.tostring = function(tr)
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

return libTransform
