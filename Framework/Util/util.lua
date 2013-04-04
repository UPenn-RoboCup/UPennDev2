module(..., package.seeall);

require('unix');
require('vector');

function ptable(t)
  -- print a table key, value pairs
  for k,v in pairs(t) do print(k,v) end
end

function parray(v, fmt)
  -- print array v according to the
  -- format string specified in fmt
  fields = {}
  fmt = fmt or '%g'
  for i,v in ipairs(v) do
    fields[#fields + 1] = string.format(fmt, v)
  end
  io.write(table.concat(fields, ' '))
  io.write('\n')
end

function mod_angle(a)
  -- Reduce angle to [-pi, pi)
  a = a % (2*math.pi);
  if (a >= math.pi) then
    a = a - 2*math.pi;
  end
  return a;
end

function sign(x)
  -- return sign of the number (-1, 0, 1)
  if (x > 0) then return 1;
  elseif (x < 0) then return -1;
  else return 0;
  end
end

function min(t)
  -- find the minimum element in the array table
  -- returns the min value and its index
  local imin = 0;
  local tmin = math.huge;
  for i = 1,#t do
    if (t[i] < tmin) then
      tmin = t[i];
      imin = i;
    end
  end
  return tmin, imin;
end

function max(t)
  local imax = 0;
  local tmax = -math.huge;
  for i = 1,#t do
    if (t[i] > tmax) then
      tmax = t[i];
      imax = i;
    end
  end
  return tmax, imax;
end

function se2_interpolate(t, u1, u2)
  -- helps smooth out the motions using a weighted average
  return vector.new{u1[1]+t*(u2[1]-u1[1]),
  u1[2]+t*(u2[2]-u1[2]),
  u1[3]+t*mod_angle(u2[3]-u1[3])};
end

function se3_interpolate(t, u1, u2, u3)
  --Interpolation between 3 xya values
  if t<0.5 then
    tt=t*2;
    return vector.new{u1[1]+tt*(u2[1]-u1[1]),
                    u1[2]+tt*(u2[2]-u1[2]),
                    u1[3]+tt*mod_angle(u2[3]-u1[3])};
  else
    tt=t*2-1;
    return vector.new{u2[1]+tt*(u3[1]-u2[1]),
                    u2[2]+tt*(u3[2]-u2[2]),
                    u2[3]+tt*mod_angle(u3[3]-u2[3])};
  end
end

function procFunc(a,deadband,maxvalue)
  --Piecewise linear function for IMU feedback
  if a>0 then
        b=math.min( math.max(0,math.abs(a)-deadband), maxvalue);
  else
        b=-math.min( math.max(0,math.abs(a)-deadband), maxvalue);
  end
  return b;
end

function pose_global(pRelative, pose)
  local ca = math.cos(pose[3]);
  local sa = math.sin(pose[3]);
  return vector.new{pose[1] + ca*pRelative[1] - sa*pRelative[2],
  pose[2] + sa*pRelative[1] + ca*pRelative[2],
  pose[3] + pRelative[3]};
end

function pose_relative(pGlobal, pose)
  local ca = math.cos(pose[3]);
  local sa = math.sin(pose[3]);
  local px = pGlobal[1]-pose[1];
  local py = pGlobal[2]-pose[2];
  local pa = pGlobal[3]-pose[3];
  return vector.new{ca*px + sa*py, -sa*px + ca*py, mod_angle(pa)};
end

function randu(n)
  --table of uniform distributed random numbers
  local t = {};
  for i = 1,n do
    t[i] = math.random();
  end
  return t;
end

function randn(n)
  -- table of normal distributed random numbers
  local t = {};
  for i = 1,n do
    --Inefficient implementation:
    t[i] = math.sqrt(-2.0*math.log(1.0-math.random())) *
    math.cos(math.pi*math.random());
  end
  return t;
end

function printf(s, ...)
  -- emulates printf function in c
  return io.write(s:format(...))
end

function loop_stats(mod)
  -- creates an object to update timing
  -- statistics from within a loop
  local t0 = unix.time()
  local count = 0
  local fps = 0
  return {
    fps = function () return fps end,
    count = function () return count end,
    update = function ()
      count = count + 1
      if (count % mod == 0) then
        local t = unix.time()
        fps = mod / (t-t0)
        t0 = t
        return true
      end
    end
  }
end
