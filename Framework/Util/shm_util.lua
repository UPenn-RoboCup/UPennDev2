require('shm')
require('carray')
require('twist')
require('vector')
require('wrench')
require('Transform')

--------------------------------------------------------------------------------
-- shm_util : utilities for initializing shared memory modules
--------------------------------------------------------------------------------

shm_util = {}

local function settings_table(value, index)
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
shm_util.settings_table = settings_table

local function shm_key_exists(shm_handle, k, nvals)
  -- checks the shm segment for the given key
  -- returns true if the key exists and is the correct (optional) length nvals

  for sk,sv in shm_handle.next, shm_handle do
    cpsv = carray.cast(shm_handle:pointer(sk))
    if (k == sk) then
      -- key exists, check length
      if (nvals and nvals ~= #cpsv) then
        return false
      else
        return true
      end
    end
  end

  -- key does not exist
  return false 
end
shm_util.shm_key_exists = shm_key_exists

local function init_shm_keys(shm_handle, shm_table)
  -- initialize a shared memory block (creating the entries if needed)
  for k,v in pairs(shm_table) do 
    -- create the key if needed
    if (type(v) == 'string') then
      if (not shm_key_exists(shm_handle, k)) then
        shm_handle:set(k, {string.byte(v, 1, string.len(v))})
      end
    elseif (type(v) == 'number') then 
      if (not shm_key_exists(shm_handle, k) or shm_handle:size(k) ~= v) then
        --local tmp = carray.new('c', v)
        --shm_handle:set(k, carray.pointer(tmp), v)
        shm_handle:empty(k, v)
      end
    elseif (type(v) == 'table') then
      if (getmetatable(v) == Transform) then
        v = Transform.get_array(v)
      end
      if (not shm_key_exists(shm_handle, k, #v)) then
        shm_handle[k] = v
      end
    end
  end
end
shm_util.init_shm_keys = init_shm_keys

local function init_shm_module(M, shm_name, shm_data, shm_size)
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
              print("NaN Detected at string!")
              return
            end
          end
          return string.char(unpack(bytes))
        end
      end
      M['set_'..k] =
      function (self, value)
        return shm_handle:set(k, {string.byte(value, 1, string.len(value))})
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
    elseif (getmetatable(v) == Transform) then
      -- Transform access 
      M['get_'..k] =
      function (self)
        local t = {}
        for i = 1,#shm_pointer[k] do
          t[i] = shm_pointer[k][i]
        end
        return Transform.array(t)
      end
      M['set_'..k] =
      function (self, value)
        local t = Transform.get_array(value)
        for i = 1,#t do
          shm_pointer[k][i] = t[i]
        end
      end
    elseif (type(v) == 'table') then
      -- array access (supports vector, twist, wrench, etc.)
      local mt = getmetatable(v)
      M['get_'..k] =
      function (self, index)
        if (type(index) == 'number') then
          return shm_pointer[k][index]
        elseif (type(index) == 'table') then
          local t = {} 
          for i = 1,#index do
            t[i] = shm_pointer[k][index[i]]
          end
          return vector.new(t)
        elseif (type(index) == 'nil') then
          local t = {}
          for i = 1,#shm_pointer[k] do
            t[i] = shm_pointer[k][i]
          end
          return setmetatable(t, mt)
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
shm_util.init_shm_module = init_shm_module

return shm_util
