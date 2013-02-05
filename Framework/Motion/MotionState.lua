require('dcm')
require('vector')
require('Config')
require('serialization')

---------------------------------------------------------
-- Motion State Base Class
---------------------------------------------------------

MotionState = {}
MotionState.__index = MotionState
MotionState.__mtstring = 'MotionState'

local joint = Config.joint

function MotionState.new(name)
  local o = {_NAME = name}
  o.dcm = dcm.new_access_point()
  o.running = false
  o.parameters = {}
  return setmetatable(o, MotionState)
end

function MotionState:set_parameter(key, value)
  -- set control parameter value
  if (type(value) == 'table') then
    for i = 1, #value do
      self.parameters[key][i] = value[i]
    end
  else
    self.parameters[key] = value
  end
end

function MotionState:set_parameters(parameters)
  -- set contol parameter values
  for k, v in pairs(parameters) do
    self:set_parameter(k, v)
  end
end

function MotionState:load_parameters(filepath)
  -- load control parameters from file
  local success, parameters = pcall(dofile, filepath)
  if (not success) then
    error(string.format('Could not load parameter file %s', filepath))
  end
  self:set_parameters(parameters)
end

function MotionState:set_joint_access(value, index)
  -- set joint write access privileges
  if value == true then value = 1 end
  if value == false then value = 0 end
  self.dcm:set_joint_write_access(value, index) 
end

function MotionState:get_parameter(key)
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

function MotionState:get_parameters()
  -- get contol parameter values
  local parameters = {}
  for k, v in pairs(self.parameters) do
    parameters[k] = self:get_parameter(k)
  end
  return parameters
end

function MotionState:save_parameters(filepath)
  local f = assert(io.open(filepath,'w+'))
  f:write('return '..serialization.serialize(self:get_parameters()))
  f:close()
end

function MotionState:get_joint_access(index)
  -- get joint write access privileges
  return self.dcm:get_joint_write_access(index)
end

function MotionState:is_running()
  -- return true if state is executing 
  return self.running 
end

function MotionState:entry()
  -- default entry
  self.running = true
end

function MotionState:update()
  -- default update
end

function MotionState:exit()
  -- default exit
  self.running = false
end

return MotionState
