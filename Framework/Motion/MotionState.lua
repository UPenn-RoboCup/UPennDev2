---------------------------------------------------------
-- Motion State Base Class
---------------------------------------------------------

require('dcm')
require('vector')
require('Config')

MotionState = {}
MotionState.__index = MotionState

local joint = Config.joint

function MotionState.new(name)
  local o = {_NAME = name}
  o.dcm = dcm.new_access_point()
  o.running = false
  o.parameters = {}
  return setmetatable(o, MotionState)
end

function MotionState:set_parameter(key, value)
  -- set controller parameter
  if (type(self.parameters[key]) == 'number') then
    self.parameters[key] = value
  elseif (type(self.parameters[key]) == 'table') then
    for i = 1,#value do
      self.parameters[key][i] = v
    end
  end
end

function MotionState:set_joint_access(value, index)
  -- set joint write access privileges
  if value == true then value = 1 end
  if value == false then value = 0 end
  self.dcm:set_joint_write_access(value, index) 
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
