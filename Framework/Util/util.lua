module(..., package.seeall);

require('shm');
require('carray');
require('vector');
require('unix');


function ptable(t)
  -- print a table key, value pairs
  for k,v in pairs(t) do print(k,v) end
end

function ptransform(t)
  --prints tranform
  for i1 = 1, 4 do 
    print(i1, t[i1][1], t[i1][2], t[i1][3], t[i1][4])
  end
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

function settings_table(value, index)
  -- return table with values mapped to index keys
  index = index or 1
  local t = {}
  if (type(index) == 'number') then
    if (type(value) == 'table') then
      for i = 1,#value do
        t[index+i-1] = value[i]
      end
    elseif (type(value) == 'number') then
      t[index] = value
    end
  elseif (type(index) == 'table') then
    if (type(value) == 'table') then
      for i = 1,#index do
        t[index[i]] = value[i]
      end
    elseif (type(value) == 'number') then
      for i = 1,#index do
        t[index[i]] = value
      end
    end
  end
  return t
end

function init_shm_module(M, shm_name, shm_data, shm_size)
  -- initialize shm segment and add accessor methods to module M
  local shm_handle = shm.new(shm_name, shm_size) 
  local shm_pointer = {}

  -- intialize shared memory
  init_shm_keys(shm_handle, shm_data)
  for k,v in pairs(shm_data) do
    shm_pointer[k] = carray.cast(shm_handle:pointer(k))
  end

  -- setup accessor methods
  for k,v in pairs(shm_data) do
    if (type(v) == 'string') then
      -- string access
      M['get_'..k] = 
      function (self)
        local bytes = shm_handle:get(k)
        if (bytes == nil) then
          return ''
        else
          for i=1,#bytes do
            if not (bytes[i]>0) then --Testing NaN
              print("NaN Detected at string!");
              return;
            end
          end
          return string.char(unpack(bytes))
        end
      end
      M['set_'..k] =
      function (self, value)
        return shm_handle:set(k, {string.byte(value, 1, string.len(value))});
      end
    elseif (type(v) == 'number') then
      -- userdata access
      M['get_'..k] =
      function (self)
        return shm_handle:pointer(k)
      end
      M['set_'..k] =
      function (self, value)
        return shm_handle:set(k, value, v)
      end
    elseif (type(v) == 'table') then
      -- number/vector access
      M['get_'..k] =
      function (self, index)
        if (type(index) == 'number') then
          return shm_pointer[k][index]
        elseif (type(index) == 'table') then
          local t = vector.new()
          for i = 1,#index do
            t[i] = shm_pointer[k][index[i]]
          end
          return t
        elseif (type(index) == 'nil') then
          local t = vector.new()
          for i = 1,#shm_pointer[k] do
            t[i] = shm_pointer[k][i]
          end
          return t
        end
      end
      M['set_'..k] =
      function (self, value, index)
        local settings = settings_table(value, index)
        for i,s in pairs(settings) do
          shm_pointer[k][i] = s
        end
      end
    else
      -- unsupported type
      error('Unsupported shm type '..type(v))
    end
  end

  -- store shm variable in module
  M[shm_name] = shm_handle
  M[shm_name..'_pointer'] = shm_ptr
end

function init_shm_segment(fenv, name, shared, shsize, tid, pid)
  tid = tid or Config.game.teamNumber;
  pid = pid or Config.game.playerID;
  -- initialize shm segments from the *cm format
  for shtable, shval in pairs(shared) do
    -- create shared memory segment
    local shmHandleName = shtable..'Shm';
    -- segment names are constructed as follows:
    -- [file_name][shared_table_name][team_number][player_id][username]
    -- ex. vcmBall01brindza is the segment for shared.ball table in vcm.lua
    -- NOTE: the first letter of the shared_table_name is capitalized
    local shmName = name..string.upper(string.sub(shtable, 1, 1))..string.sub(shtable, 2)..tid..pid..(os.getenv('USER') or '');

    fenv[shmHandleName] = shm.new(shmName, shsize[shtable]);
    local shmHandle = fenv[shmHandleName];

    -- intialize shared memory
    init_shm_keys(shmHandle, shared[shtable]);

    -- generate accessors and pointers
    local shmPointerName = shtable..'Ptr';
    fenv[shmPointerName] = {};
    local shmPointer = fenv[shmPointerName];

    for k,v in pairs(shared[shtable]) do
      shmPointer[k] = carray.cast(shmHandle:pointer(k));
      if (type(v) == 'string') then
        -- setup accessors for a string
        fenv['get_'..shtable..'_'..k] =
        function()
          local bytes = shmHandle:get(k);
          if (bytes == nil) then
            return '';
          else
            for i=1,#bytes do
              if not (bytes[i]>0) then --Testing NaN
                print("NaN Detected at string!");
                return;
              end
            end
            return string.char(unpack(bytes));
          end
        end
        fenv['set_'..shtable..'_'..k] =
        function(val)
          return shmHandle:set(k, {string.byte(val, 1, string.len(val))});
        end
      elseif (type(v) == 'number') then
        -- setup accessors for a userdata
        fenv['get_'..shtable..'_'..k] =
        function()
          return shmHandle:pointer(k);
        end
        fenv['set_'..shtable..'_'..k] =
        function(val)
          return shmHandle:set(k, val, v);
        end
      elseif (type(v) == 'table') then
        -- setup accessors for a number/vector 
        fenv['get_'..shtable..'_'..k] =
        function()
          val = shmHandle:get(k);
          if type(val) == 'table' then
            val = vector.new(val);
          end
          return val;
        end
        fenv['set_'..shtable..'_'..k] =
        function(val, ...)
          return shmHandle:set(k, val, ...);
        end
      else
        -- unsupported type
        error('Unsupported shm type '..type(v));
      end
    end
  end
end


function init_shm_keys(shmHandle, shmTable)
  -- initialize a shared memory block (creating the entries if needed)
  for k,v in pairs(shmTable) do 
    -- create the key if needed
    if (type(v) == 'string') then
      if (not shm_key_exists(shmHandle, k)) then
        shmHandle:set(k, {string.byte(v, 1, string.len(v))});
      end
    elseif (type(v) == 'number') then 
      if (not shm_key_exists(shmHandle, k) or shmHandle:size(k) ~= v) then
        --local tmp = carray.new('c', v);
        --shmHandle:set(k, carray.pointer(tmp), v);
        shmHandle:empty(k, v);
      end
    elseif (type(v) == 'table') then
      if (not shm_key_exists(shmHandle, k, #v)) then
        shmHandle[k] = v;
      end
    end
  end
end

function shm_key_exists(shmHandle, k, nvals)
  -- checks the shm segment for the given key
  -- returns true if the key exists and is of the correct length nvals (if provided)

  for sk,sv in shmHandle.next, shmHandle do
    cpsv = carray.cast(shmHandle:pointer(sk));
    if (k == sk) then
      -- key exists, check length
      if (nvals and nvals ~= #cpsv) then
        return false;
      else
        return true;
      end
    end
  end

  -- key does not exist
  return false; 
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
