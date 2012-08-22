require('vector')
require('Config')

Motion = {}

local joint = Config.joint
local state_machines = {} -- MotionFSM's

function Motion.add_fsm(sm)
  state_machines[#state_machines+1] = sm
  sm:entry()
end

function Motion.remove_fsm(sm)
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
  for i,sm in ipairs(state_machines) do
    if (sm:is_active()) then
      joint_access = joint_access + vector.new(sm:get_joint_access())
    end
  end
  for i = 1,#joint_access do
    assert(joint_access[i] <= 1, 
      string.format('multiple motion states writing to joint.%s', joint.id[i]))
  end
end

function Motion.entry()
  Motion.check_joint_access()
  for i,sm in ipairs(state_machines) do
    sm:entry()
  end
end

function Motion.update()
  Motion.check_joint_access()
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
