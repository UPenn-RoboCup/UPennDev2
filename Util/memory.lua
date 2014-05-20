-- TODO: Make an FFI version (may be faster?)
-- TODO: Just replace carray with FFI?
--
local shm    = require'shm'
local vector = require'vector'
local carray
if not ffi then carray = require'carray' end

local memory = {}

local function init_shm_keys (shmHandle, shmTable)
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

function memory.init_shm_segment (name, shared, shsize, tid, pid)
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
    init_shm_keys(shmHandle, shared[shtable]);

		-- Add more direct memory access
		-- Default is LuaJIT FFI, with carray fallback
		local shmPointerName = shtable..'Ptr';
		fenv[shmPointerName] = {};
		local shmPointer = fenv[shmPointerName]
    for k,v in pairs(shared[shtable]) do
			local ptr, tp, n = shmHandle:pointer(k)

      -- If FFI, then give raw access to the SHM pointer
			if ffi then
				shmPointer[k] = ffi.cast(tp..'*', ptr)
			--else
			--	shmPointer[k] = carray.cast(ptr, tp, n)
			end
      local kind = type(v)
      if kind=='string' then
        -- Get String
        fenv['get_'..shtable..'_'..k] = function()
            local bytes = shmHandle:get(k)
            if not bytes then return '' end
            for i,b in ipairs(bytes) do
              --Testing NaN
              if not (b>0) then print("NaN in str!",b,i) return end
            end
            return string.char(unpack(bytes));
          end
        -- Set string. Never trust FFI here... since expansion may need to happen
        fenv['set_'..shtable..'_'..k] = function(val)
          return shmHandle:set(k, {string.byte(val, 1, string.len(val))});
        end

      elseif kind=='number' then
        -- Get userdata
        fenv['get_'..shtable..'_'..k] = function() return shmHandle:pointer(k) end
        -- Set userdata (Can accept a string for memcpy'ing the string)
        fenv['set_'..shtable..'_'..k] = function(val) return shmHandle:set(k, val, v) end

      elseif kind=='table' then
        -- setup accessors for a number/vector
        fenv['get_'..shtable..'_'..k] = function()
          local val = shmHandle:get(k)
          if type(val) == 'table' then val = vector.new(val) end
          return val
        end
        -- Set table
        fenv['set_'..shtable..'_'..k] = function(val, ...)
          return shmHandle:set(k, val, ...)
        end
        --[[
        -- Delta table if we have a vector of numbers
        fenv['delta_'..shtable..'_'..k] = function(delta, ...)
          local val = shmHandle:get(k)
          local tv = type(val)
          -- Must both be vectors
          if tv~=type(delta) then return end
          val = vector.new(val)
          delta = vector.new(delta)
          -- Must be the same size
          if tv=='table' and #val~=#delta then return end
          -- Set in memory
          return shmHandle:set(k, val + delta, ...)
        end
        --]]
      else
        -- unsupported type
        error('Unsupported shm type: '..kind);
      end
    end
  end
end

function memory.shm_key_exists(shmHandle, k, nvals)
  -- checks the shm segment for the given key
  -- returns true if the key exists and is of the correct length nvals (if provided)
  for sk,sv in shmHandle.next, shmHandle do
		local ptr, tp, n = shmHandle:pointer(sk)
    if k == sk then
      -- key exists, check length
			if (nvals and nvals ~= n) then
        return false
      else
        return true
      end
    end
  end

  -- key does not exist
  return false;
end
return memory
