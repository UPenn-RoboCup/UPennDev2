require('dcm')
require('vector')
require('Config')
require('serialization')

---------------------------------------------------------
-- Motion State Base Class
---------------------------------------------------------

Motion_state = {}
Motion_state.__index = Motion_state
Motion_state.__mtstring = 'Motion_state'

local joint = Config.joint

function Motion_state.new(name)
  local o = {_NAME = name}
  o.dcm = dcm.new_access_point()
  o.parameters = {}
  return setmetatable(o, Motion_state)
end

function Motion_state:set_parameter(key, value)
  -- set control parameter value
  if (type(value) == 'table') then
    for i = 1, #value do
      self.parameters[key][i] = value[i]
    end
  else
    self.parameters[key] = value
  end
end

function Motion_state:set_parameters(parameters)
  -- set contol parameter values
  for k, v in pairs(parameters) do
    self:set_parameter(k, v)
  end
end

function Motion_state:load_parameters(filepath)
  -- load control parameters from file
  local success, parameters = pcall(dofile, filepath)
  if (not success) then
    error(string.format('Could not load parameter file %s', filepath))
  end
  self:set_parameters(parameters)
end

function Motion_state:set_joint_access(value, index)
  -- set joint write access privileges
  if value == true then value = 1 end
  if value == false then value = 0 end
  self.dcm:set_joint_write_access(value, index) 
end

function Motion_state:get_parameter(key)
  -- get control parameter value
  local value = self.parameters[key]
  if (type(value) == 'table') then
    local v = {}
    for i = 1, #value do
      v[i] = value[i]
    end
    return v
  else 
    return value
  end
end

function Motion_state:get_parameters()
  -- get contol parameter values
  local parameters = {}
  for k, v in pairs(self.parameters) do
    parameters[k] = self:get_parameter(k)
  end
  return parameters
end

function Motion_state:save_parameters(filepath)
  local f = assert(io.open(filepath,'w+'))
  f:write('return '..serialization.serialize(self:get_parameters()))
  f:close()
end

function Motion_state:get_joint_access(index)
  -- get joint write access privileges
  return self.dcm:get_joint_write_access(index)
end

function Motion_state:entry()
  -- default entry
end

function Motion_state:update()
  -- default update
end

function Motion_state:exit()
  -- default exit
end

return Motion_state
