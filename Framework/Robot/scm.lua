require('Config')
require('carray')
require('vector')
require('util')
require('shm')

---------------------------------------------------------------------------
-- scm : sensor communication manager
---------------------------------------------------------------------------

scm = {}

-- configure data groups and keys
---------------------------------------------------------------------------

local groups = {
  joint = {
    'joint_position',
    'joint_force'
  },
  motor = {
    'motor_position',
    'motor_force',
    'motor_current',
    'motor_temperature'
  },
  force_torque = {
    'force_torque'
  },
  tactile_array = {
    'tactile_array'
  },
  ahrs = {
    'ahrs'
  },
  battery = {
    'battery'
  }
}

local key_group = {}
scm.group_size = {}
scm.group_index = {}
scm.group_write_access = {}

for group in pairs(groups) do
  for _,key in pairs(groups[group]) do
    key_group[key] = group
  end
end

for group in pairs(groups) do
  scm.group_size[group] = Config[group] and #Config[group].id or 1
  scm.group_index[group] = Config[group] and Config[group].index or {}
  scm.group_write_access[group] = vector.ones(scm.group_size[group])
end

-- configure shared memory
---------------------------------------------------------------------------

-- define shared data fields and update flags 
local shared_data = {}
for key,group in pairs(key_group) do
  shared_data[key] = vector.zeros(scm.group_size[group])
  shared_data[key..'_updated'] = vector.zeros(scm.group_size[group])
end

-- initialize shared memory segment
scm.shm = shm.new('sensor')
util.init_shm_keys(scm.shm, shared_data)

-- get array access to shared data 
local data = {}
for key in pairs(shared_data) do
  data[key] = carray.cast(scm.shm:pointer(key))
end

-- define access methods
---------------------------------------------------------------------------

local function get_settings_table(value, index)
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

function scm:set_key(key, value, index)
  local group = key_group[key]
  if (not group) then
    return false
  end
  local group_index = self.group_index[group]
  local group_write_access = self.group_write_access[group]
  if (type(index) == 'string') then
    index = group_index[index]
  end
  local key_updated = key..'_updated'
  local settings = get_settings_table(value, index)
  for i,v in pairs(settings) do
    if (group_write_access[i] == 1) then
      data[key][i] = v
      data[key_updated][i] = 1
    end
  end
  return true
end

function scm:get_key(key, index)
  local group = key_group[key]
  if (not group) then
    return nil
  end
  local group_index = self.group_index[group]
  if (type(index) == 'string') then
    index = group_index[index]
  end
  if (type(index) == 'number') then
    return data[key][index]
  elseif (type(index) == 'table') then
    local t = vector.new()
    for i = 1,#index do
      t[i] = data[key][index[i]]
    end
    return t
  elseif (type(index) == 'nil') then
    local t = vector.new()
    for i = 1,#data[key] do
      t[i] = data[key][i]
    end
    return t
  end
end

function scm:set_write_access(key, value, index)
  local group = groups[key] and key or key_group[key]
  if (not group) then
    return false
  end
  local group_index = self.group_index[group]
  local group_write_access = self.group_write_access[group]
  if (type(index) == 'string') then
    index = group_index[index]
  end
  local settings = get_settings_table(value, index)
  for i,v in pairs(settings) do
    group_write_access[i] = v
  end
  return true
end

function scm:get_write_access(key, value, index)
  local group = groups[key] and key or key_group[key]
  if (not group) then
    return nil
  end
  local group_index = self.group_index[group]
  local group_write_access = self.group_write_access[group]
  if (type(index) == 'string') then
    index = group_index[index]
  end

  if (type(index) == 'number') then
    return group_write_access[index]
  elseif (type(index) == 'table') then
    local t = vector.new()
    for i = 1,#index do
      t[i] = group_write_access[index[i]]
    end
    return t
  elseif (type(index) == 'nil') then
    local t = vector.new()
    for i = 1,#group_write_access do
      t[i] = group_write_access[i]
    end
    return t
  end
end

function scm:get_group(key)
  return key_group[key]
end

for key,group in pairs(key_group) do
  scm['set_'..key] =
    function (self, value, index)
      return self:set_key(key, value, index)
    end
  scm["get_"..key] =
    function (self, index)
      return self:get_key(key, index)
    end
  scm['set_'..key..'_write_access'] =
    function (self, value, index)
      return self:set_write_access(group, value, index)
    end
  scm['get_'..key..'_write_access'] =
    function (self, index)
      return self:get_write_access(group, index)
    end
  scm['get_'..key..'_group'] = 
    function (self, index)
      return group
    end
end

for group in pairs(groups) do
  scm['set_'..group..'_write_access'] =
    function (self, value, index)
      return self:set_write_access(group, value, index)
    end
  scm['get_'..group..'_write_access'] =
    function (self, index)
      return self:get_write_access(group, index)
    end
end

-- define constructor for instances with private write access
---------------------------------------------------------------------------

function scm.new_access_point()
  local o = {}
  o.group_write_access = {}
  for k,v in pairs(scm.group_write_access) do
    o.group_write_access[k] = vector.zeros(#v) -- access disabled by default
  end
  return setmetatable(o, {__index = scm})
end

return scm
