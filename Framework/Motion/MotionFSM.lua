---------------------------------------------------------
-- Motion State Machine Base Class
---------------------------------------------------------

require('fsm')

MotionFSM = {}
MotionFSM.__index = MotionFSM
setmetatable(MotionFSM, fsm)

function MotionFSM.new(initialState, ...)
  -- inheret from fsm class
  local o = fsm.new(initialState, ...) 
  return setmetatable(o, MotionFSM)
end

function MotionFSM:set_joint_access(value, index)
  -- set joint write access priveleges for each MotionState
  for i,state in ipairs(self.states) do
    state:set_joint_access(value, index)
  end
end

function MotionFSM:get_joint_access(index)
  -- get joint write access priveleges for current MotionState
  local current_state = self:get_current_state()
  return current_state:get_joint_access(index) 
end

function MotionFSM:event(event)
  -- add new fsm event
  self:add_event(event)
end

return MotionFSM
