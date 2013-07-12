-- Utility Functions

require 'torch'
-- TODO: Double*Float=???
torch.Tensor = torch.DoubleTensor

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

return libTrig
