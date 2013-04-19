require('fsm')

---------------------------------------------------------
-- Motion State Machine Base Class
---------------------------------------------------------

Motion_fsm = {}

Motion_fsm.__index = Motion_fsm
Motion_fsm.__mtstring = 'Motion_fsm'
setmetatable(Motion_fsm, fsm)

function Motion_fsm.new(initialState, ...)
  -- inheret from fsm class
  local o = fsm.new(initialState, ...) 
  return setmetatable(o, Motion_fsm)
end

function Motion_fsm:set_joint_access(value, index)
  -- set joint write access priveleges for each Motion_state
  for i,state in ipairs(self.states) do
    state:set_joint_access(value, index)
  end
end

function Motion_fsm:get_joint_access(index)
  -- get joint write access priveleges for current Motion_state
  local current_state = self:get_current_state()
  return current_state:get_joint_access(index) 
end

return Motion_fsm
