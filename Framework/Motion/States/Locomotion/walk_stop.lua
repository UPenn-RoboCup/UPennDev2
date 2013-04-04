require('Motion_state')
require('Config')
require('walk')

--------------------------------------------------------------------------
-- Walk Stop Controller
--------------------------------------------------------------------------

walk_stop = Motion_state.new('walk_stop')

local dcm = walk_stop.dcm
local ahrs = Config.ahrs
local joint = Config.joint
walk_stop:set_joint_access(0, joint.all)
walk_stop:set_joint_access(1, joint.lowerbody)

-- default parameters
walk_stop.parameters = {
}

-- config parameters
walk_stop:load_parameters()

function walk_stop:entry()
  walk:stop()
end

function walk_stop:update()
  return walk:update()
end

function walk_stop:exit()
  walk:exit()
end

return walk_stop
