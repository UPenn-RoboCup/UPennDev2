require('vector')
require('Config')
require('mcm')

---------------------------------------------------------
-- Motion
---------------------------------------------------------

Motion = {}

local joint = Config.joint
local state_machines = {} -- Motion_fsm's

function Motion.add_fsm(sm)
  if (type(sm) == 'string') then
    sm = loadstring('return '..sm)()
  end
  if (type(sm) == 'table') then
    state_machines[#state_machines+1] = sm
    sm:entry()
  end
end

function Motion.remove_fsm(sm)
  if (type(sm) == 'string') then
    sm = loadstring('return '..sm)()
  end
  for i,smi in ipairs(state_machines) do
    if (smi == sm) then
      table.remove(state_machines, i)
      sm:exit()
      break
    end
  end
end

function Motion.check_joint_access()
  local joint_access = vector.zeros(#joint.id)
  for i = 1,#state_machines do
    local sm = state_machines[i]
    joint_access = joint_access + vector.new(sm:get_joint_access())
  end
  for i = 1,#joint_access do
    if (joint_access[i] > 1) then
      return false
    end
  end
  return true
end

function Motion.get_states()
  local states = {}
  for i = 1,#state_machines do
    local sm = state_machines[i]
    local sm_states = sm:get_states()
    for j = 1,#sm_states do
      states[#states + 1] = sm_states[j]
    end
  end
  return states
end

function Motion.entry()
  for i,sm in ipairs(state_machines) do
    sm:entry()
  end
end

function Motion.update()
  for i,sm in ipairs(state_machines) do
    sm:update()
  end
end

function Motion.exit() 
  for i,sm in ipairs(state_machines) do
    sm:exit()
  end
end

return Motion
