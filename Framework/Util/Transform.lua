module(..., package.seeall);

require('vector');

mt = {};

-- Constructors
--------------------------------------------------------------------------

function eye()
  local t = {};
  t[1] = vector.new({1, 0, 0, 0});
  t[2] = vector.new({0, 1, 0, 0});
  t[3] = vector.new({0, 0, 1, 0});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, mt);
end

function rotZ(a)
  local ca = math.cos(a);
  local sa = math.sin(a);
  local t = {};
  t[1] = vector.new({ca, -sa, 0, 0});
  t[2] = vector.new({sa, ca, 0, 0});
  t[3] = vector.new({0, 0, 1, 0});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, mt);
end

function rotY(a)
  local ca = math.cos(a);
  local sa = math.sin(a);
  local t = {};
  t[1] = vector.new({ca, 0, sa, 0});
  t[2] = vector.new({0, 1, 0, 0});
  t[3] = vector.new({-sa, 0, ca, 0});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, mt);
end

function rotX(a)
  local ca = math.cos(a);
  local sa = math.sin(a);
  local t = {};
  t[1] = vector.new({1, 0, 0, 0});
  t[2] = vector.new({0, ca, -sa, 0});
  t[3] = vector.new({0, sa, ca, 0});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, mt);
end

function transX(dx)
  local t = {};
  t[1] = vector.new({1, 0, 0, dx});
  t[2] = vector.new({0, 1, 0, 0});
  t[3] = vector.new({0, 0, 1, 0});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, mt);
end

function transY(dy)
  local t = {};
  t[1] = vector.new({1, 0, 0, 0});
  t[2] = vector.new({0, 1, 0, dy});
  t[3] = vector.new({0, 0, 1, 0});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, mt);
end

function transZ(dz)
  local t = {};
  t[1] = vector.new({1, 0, 0, 0});
  t[2] = vector.new({0, 1, 0, 0});
  t[3] = vector.new({0, 0, 1, dz});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, mt);
end

function trans(dx, dy, dz)
  local t = {};
  t[1] = vector.new({1, 0, 0, dx});
  t[2] = vector.new({0, 1, 0, dy});
  t[3] = vector.new({0, 0, 1, dz});
  t[4] = vector.new({0, 0, 0, 1});
  return setmetatable(t, mt);
end

function transform6D(p)
  local t = Transform.eye(); 
  --[[
  t = Transform.trans(p[1], p[2], p[3])
  t = Transform.rotX(p[4])*t
  t = Transform.rotY(p[5])*t
  t = Transform.rotZ(p[6])*t
  --]]
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
  return setmetatable(t, mt);
end

-- Methods
--------------------------------------------------------------------------

function inv(t)
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

function position6D(t)
  local p = vector.zeros(6);
  local w = getEuler(t);
  p[1] = t[1][4];
  p[2] = t[2][4];
  p[3] = t[3][4];
  p[4] = w[1];
  p[5] = w[2];
  p[6] = w[3];
  return p;
end

function getEuler(t)
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

function tostring(t)
  local str = ''
  for i = 1,4 do
    for j = 1,4 do
      str = str..string.format('%.4f ', t[i][j]);
    end
    str = str..'\n';
  end
  return str;
end

mt.__index = getfenv();
mt.__tostring = tostring;
mt.__mul = function(t1, t2)
  local t = {};
  if (type(t2[1]) == "number") then
    for i = 1,4 do
      t[i] = t1[i][1] * t2[1]
              + t1[i][2] * t2[2]
              + t1[i][3] * t2[3]
              + t1[i][4] * t2[4];
    end
    return vector.new(t);
  elseif (type(t2[1] == "table")) then
    for i = 1,4 do
      t[i] = {};
      for j = 1,4 do
        t[i][j] = t1[i][1] * t2[1][j]
                  + t1[i][2] * t2[2][j]
                  + t1[i][3] * t2[3][j]
                  + t1[i][4] * t2[4][j];
      end
    end
    return setmetatable(t, mt);
  end
end
