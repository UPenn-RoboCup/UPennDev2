-- Transformation Matrix functions
local torch = require'torch'
torch.Tensor = torch.DoubleTensor
local quaternion = require'quaternion'

-- TODO: Is this actually a good name?
local libTrig = {}

-- TODO: Make sure the helper functions are working properly!
libTrig.rotx = function(t)
  -- Homogeneous transformation representing a rotation of theta
  -- about the X axis.
  local ct = math.cos(t);
  local st = math.sin(t);
  local r = torch.eye(4);
  r[2][2] = ct;
  r[3][3] = ct;
  r[2][3] = -1*st;
  r[3][2] = st;
  return r
end

libTrig.roty = function(t)
  -- Homogeneous transformation representing a rotation of theta
  -- about the Y axis.
  local ct = math.cos(t);
  local st = math.sin(t);
  local r = torch.eye(4);
  r[1][1] = ct;
  r[3][3] = ct;
  r[1][3] = st;
  r[3][1] = -1*st;
  return r
end

libTrig.rotz = function(t)
  -- Homogeneous transformation representing a rotation of theta
  -- about the Z axis.
  local ct = math.cos(t);
  local st = math.sin(t);
  local r = torch.eye(4);
  r[1][1] = ct;
  r[2][2] = ct;
  r[1][2] = -1*st;
  r[2][1] = st;
  return r
end

libTrig.trans = function(v)
  local t = torch.eye(4);
  t[1][4] = v[1] or 0;
  t[2][4] = v[2] or 0;
  t[3][4] = v[3] or 0;
  return t;
end

-- Rotation matrix to Roll Pitch Yaw
-- Yida's from http://planning.cs.uiuc.edu/node102.html
libTrig.to_rpy = function( R )
  local y = math.atan2(R[2][1], R[1][1])
  local p = math.atan2(-R[3][1], math.sqrt(R[3][2]^2+R[3][3]^2))
  local r = math.atan2(R[3][2], R[3][3])
  return {r,p,y}
end

-- From Yida, with a resourse
-- http://planning.cs.uiuc.edu/node102.html
libTrig.from_rpy = function( rpy )
  local alpha = rpy[3]
  local beta = rpy[2]
  local gamma = rpy[1]
  R[1][1] = math.cos(alpha
  R[1][1] = math.cos(alpha) * math.cos(beta)
  R[2][1] = math.sin(alpha) * math.cos(beta)
  R[3][1] = -math.sin(beta)
  R[1][2] = math.cos(alpha) * math.sin(beta) * math.sin(gamma) - math.sin(alpha) * math.cos(gamma)
  R[2][2] = math.sin(alpha) * math.sin(beta) * math.sin(gamma) + math.cos(alpha) * math.cos(gamma)
  R[3][2] = math.cos(beta) * math.sin(gamma)
  R[1][3] = math.cos(alpha) * math.sin(beta) * math.cos(gamma) + math.sin(alpha) * math.sin(gamma)
  R[2][3] = math.sin(alpha) * math.sin(beta) * math.cos(gamma) - math.cos(alpha) * math.sin(gamma)
  R[3][3] = math.cos(beta) * math.cos(gamma)
  return R
end

-- Rotation Matrix to quaternion
-- from Yida
libTrig.to_quaternion = function(R)
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

function libTrig.from_quaternion( q )
  local R = torch.DoubleTensor(3,3):fill(0)
  R[1][1] = 1 - 2 * q[3] * q[3] - 2 * q[4] * q[4]
  R[1][2] = 2 * q[2] * q[3] - 2 * q[4] * q[1]
  R[1][3] = 2 * q[2] * q[4] + 2 * q[3] * q[1]
  R[2][1] = 2 * q[2] * q[3] + 2 * q[4] * q[1]
  R[2][2] = 1 - 2 * q[2] * q[2] - 2 * q[4] * q[4]
  R[2][3] = 2 * q[3] * q[4] - 2 * q[2] * q[1]
  R[3][1] = 2 * q[2] * q[4] - 2 * q[3] * q[1]
  R[3][2] = 2 * q[3] * q[4] + 2 * q[2] * q[1]
  R[3][3] = 1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]
  return R
end

return libTrig