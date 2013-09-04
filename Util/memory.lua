local shm    = require'shm'
local carray = require'carray'
local vector = require'vector'

local memory = {}

function memory.init_shm_segment(name, shared, shsize, tid, pid)
  local tid = tid or 0
  local pid = pid or 1

  local fenv = _G[name]
  if fenv == nil then
    _G[name] = {}
    fenv = _G[name]
  end

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
    memory.init_shm_keys(shmHandle, shared[shtable]);

    -- generate accessors and pointers
    local shmPointerName = shtable..'Ptr';
    fenv[shmPointerName] = {};
    local shmPointer = fenv[shmPointerName]

    for k,v in pairs(shared[shtable]) do
      shmPointer[k] = carray.cast(shmHandle:pointer(k))
      
      if type(v)=='string' then
        -- Get String
        fenv['get_'..shtable..'_'..k] = function()
            local bytes = shmHandle:get(k);
            if bytes == nil then
              return ''
            else
              for i=1,#bytes do
                --Testing NaN
                if not (bytes[i]>0) then 
                  print("NaN Detected at string!") return
                end
              end
              return string.char(unpack(bytes));
            end
          end
        -- Set string
        fenv['set_'..shtable..'_'..k] = function(val)
          return shmHandle:set(k, {string.byte(val, 1, string.len(val))});
        end
        
      elseif type(v)=='number' then
        -- Get userdata
        fenv['get_'..shtable..'_'..k] = function() return shmHandle:pointer(k) end
        -- Set userdata (Can accept a string for memcpy'ing the string)
        fenv['set_'..shtable..'_'..k] = function(val) return shmHandle:set(k, val, v) end
        
      elseif type(v)=='table' then
        -- setup accessors for a number/vector 
        fenv['get_'..shtable..'_'..k] = function()
          val = shmHandle:get(k)
          if type(val) == 'table' then val = vector.new(val) end
          return val
        end
        -- Set table
        fenv['set_'..shtable..'_'..k] = function(val, ...)
          return shmHandle:set(k, val, ...)
        end

      else
        -- unsupported type
        error('Unsupported shm type '..type(v));
      end
    end
  end
end

function memory.init_shm_keys(shmHandle, shmTable)
  -- initialize a shared memory block (creating the entries if needed)
  for k,v in pairs(shmTable) do 
    -- create the key if needed
    if type(v) == 'string' then
      if not memory.shm_key_exists(shmHandle, k) then
        shmHandle:set(k, {string.byte(v, 1, string.len(v))});
      end
    elseif type(v) == 'number' then 
      if not memory.shm_key_exists(shmHandle, k) or shmHandle:size(k) ~= v then
        shmHandle:empty(k, v);
      end
    elseif type(v) == 'table' then
      if not memory.shm_key_exists(shmHandle, k, #v) then
        shmHandle[k] = v;
      end
    end
  end
end

function memory.shm_key_exists(shmHandle, k, nvals)
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
return memory