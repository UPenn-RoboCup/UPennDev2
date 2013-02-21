require('vector');
require('wrench');
require('twist');

----------------------------------------------------------------------
-- Transform : homogeneous transform
----------------------------------------------------------------------

Transform = {};
Transform.__index = Transform;
Transform.__mtstring = 'Transform';

-- Constructors
--------------------------------------------------------------------------

function Transform.eye()
  local t = {};
  t[1] = vector.new({1, 0, 0, 0});
  t[2] = vector.new({0, 1, 0, 0});
  t[3] = vector.new({0, 0, 1, 0});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, Transform);
end

function Transform.rotZ(a)
  local ca = math.cos(a);
  local sa = math.sin(a);
  local t = {};
  t[1] = vector.new({ca, -sa, 0, 0});
  t[2] = vector.new({sa, ca, 0, 0});
  t[3] = vector.new({0, 0, 1, 0});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, Transform);
end

function Transform.rotY(a)
  local ca = math.cos(a);
  local sa = math.sin(a);
  local t = {};
  t[1] = vector.new({ca, 0, sa, 0});
  t[2] = vector.new({0, 1, 0, 0});
  t[3] = vector.new({-sa, 0, ca, 0});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, Transform);
end

function Transform.rotX(a)
  local ca = math.cos(a);
  local sa = math.sin(a);
  local t = {};
  t[1] = vector.new({1, 0, 0, 0});
  t[2] = vector.new({0, ca, -sa, 0});
  t[3] = vector.new({0, sa, ca, 0});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, Transform);
end

function Transform.transX(dx)
  local t = {};
  t[1] = vector.new({1, 0, 0, dx});
  t[2] = vector.new({0, 1, 0, 0});
  t[3] = vector.new({0, 0, 1, 0});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, Transform);
end

function Transform.transY(dy)
  local t = {};
  t[1] = vector.new({1, 0, 0, 0});
  t[2] = vector.new({0, 1, 0, dy});
  t[3] = vector.new({0, 0, 1, 0});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, Transform);
end

function Transform.transZ(dz)
  local t = {};
  t[1] = vector.new({1, 0, 0, 0});
  t[2] = vector.new({0, 1, 0, 0});
  t[3] = vector.new({0, 0, 1, dz});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, Transform);
end

function Transform.rotation(r)
  local t = {}
  assert(#r == 9);
  t[1] = vector.new({r[1], r[2], r[3], 0});
  t[2] = vector.new({r[4], r[5], r[6], 0});
  t[3] = vector.new({r[7], r[8], r[9], 0});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, Transform);
end

function Transform.translation(p)
  local t = {};
  assert(#p == 3);
  t[1] = vector.new({1, 0, 0, p[1]});
  t[2] = vector.new({0, 1, 0, p[2]});
  t[3] = vector.new({0, 0, 1, p[3]});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, Transform);
end

function Transform.pose(p)
  --[[
  return transform :
  t = Transform.transX(p[1])
  t = Transform.transY(p[2])
  t = Transform.transZ(p[3])
  t = t*Transform.rotX(p[4])
  t = t*Transform.rotY(p[5])
  t = t*Transform.rotZ(p[6])
  --]]
  local t = Transform.eye(); 
  for i = 4,6 do
    p[i] = p[i] or 0;
  end
  assert(#p == 6);
  local cwx = math.cos(p[4]);
  local swx = math.sin(p[4]);
  local cwy = math.cos(p[5]);
  local swy = math.sin(p[5]);
  local cwz = math.cos(p[6]);
  local swz = math.sin(p[6]);
  t[1][1] = cwy*cwz; 
  t[1][2] = -cwy*swz;
  t[1][3] = swy; 
  t[1][4] = p[1];
  t[2][1] = cwx*swz + swx*swy*cwz;
  t[2][2] = cwx*cwz - swx*swy*swz;
  t[2][3] = -swx*cwy;
  t[2][4] = p[2]; 
  t[3][1] = swx*swz - cwx*swy*cwz;
  t[3][2] = swx*cwz + cwx*swy*swz; 
  t[3][3] = cwx*cwy; 
  t[3][4] = p[3]; 
  t[4][1] = 0; 
  t[4][2] = 0; 
  t[4][3] = 0; 
  t[4][4] = 1; 
  return setmetatable(t, Transform);
end

function Transform.euler(w)
  --[[
  return transform :
  t = t*Transform.rotX(w[1])
  t = t*Transform.rotY(w[2])
  t = t*Transform.rotZ(w[3])
  --]]
  assert(#w == 3);
  return Transform.pose({0, 0, 0, w[1], w[2], w[3]});
end

-- Methods
--------------------------------------------------------------------------

function Transform.inv(t)
  tinv = Transform.eye();
  for i = 1,3 do 
    for j = 1,3 do
      -- Transpose rotation:
      tinv[i][j] = t[j][i];
      -- Compute inv translation:
      tinv[i][4] = tinv[i][4] - t[j][i]*t[j][4];
    end
  end
  return tinv;
end

function Transform.get_rotation(t)
  local r = {};
  r[1] = t[1][1];
  r[2] = t[1][2];
  r[3] = t[1][3];
  r[4] = t[2][1];
  r[5] = t[2][2];
  r[6] = t[2][3];
  r[7] = t[3][1];
  r[8] = t[3][2];
  r[9] = t[3][3];
  return vector.new(r);
end

function Transform.get_translation(t)
  local p = {}; 
  p[1] = t[1][4];
  p[2] = t[2][4];
  p[3] = t[3][4];
  return vector.new(p);
end

function Transform.get_pose(t)
  -- returns 6 DOF translation and rotation {tx, ty, tz, rx, ry, rz}
  local p = vector.zeros(6);
  local w = Transform.get_euler(t);
  p[1] = t[1][4];
  p[2] = t[2][4];
  p[3] = t[3][4];
  p[4] = w[1];
  p[5] = w[2];
  p[6] = w[3];
  return p;
end

function Transform.get_euler(t)
  -- returns euler angles {wx, wy, wz} corresponding to rotation RxRyRz
  local w = vector.zeros(3);
  w[2] = math.asin(t[1][3]);
  if (t[1][3] == -1) then
    w[1] = -math.atan2(t[2][1], t[2][2]);
    w[3] = 0;
  elseif (t[1][3] == 1) then
    w[1] = math.atan2(t[2][1], t[2][2]);
    w[3] = 0;
  else
    w[1] = math.atan2(-t[2][3]/math.cos(w[2]), t[3][3]/math.cos(w[2])); 
    w[3] = math.atan2(-t[1][2]/math.cos(w[2]), t[1][1]/math.cos(w[2]));
  end
  return w;
end

function Transform.__tostring(t)
  local str = ''
  for i = 1,4 do
    for j = 1,4 do
      str = str..string.format('%.4f ', t[i][j]);
    end
    str = str..'\n';
  end
  return str;
end

function Transform.__mul(t1, t2)
  if (getmetatable(t2) == vector) then
    local v = {};
    for i = 1,4 do
      v[i] = t1[i][1] * t2[1]
              + t1[i][2] * t2[2]
              + t1[i][3] * t2[3]
              + t1[i][4] *(t2[4] or 1);
    end
    for i = 1,4 do
      v[i] = v[i]/v[4];
    end
    return setmetatable(v, vector);
  elseif (getmetatable(t2) == Transform) then
    local t = {};
    for i = 1,4 do
      t[i] = {};
      for j = 1,4 do
        t[i][j] = t1[i][1] * t2[1][j]
                  + t1[i][2] * t2[2][j]
                  + t1[i][3] * t2[3][j]
                  + t1[i][4] * t2[4][j];
      end
    end
    return setmetatable(t, Transform);
  elseif (getmetatable(t2) == wrench) then
    local M = Transform.rotation(t1:get_rotation());
    local p = t1:get_translation();
    local w = wrench.new();
    w:set_force(M*t2:get_force());
    w:set_torque(M*t2:get_torque());
    return w:translate(-p);
  elseif (getmetatable(t2) == twist) then
    local M = Transform.rotation(t1:get_rotation());
    local p = t1:get_translation();
    local t = twist.new();
    t:set_lin(M*t2:get_lin());
    t:set_rot(M*t2:get_rot());
    return t:translate(-p);
  else
    error('attempt to multiply Transform by incompatible type');
  end
end

return Transform;
