require('Motion_state')
require('Config')
require('walk')

--------------------------------------------------------------------------
-- Walk Stop Controller
--------------------------------------------------------------------------

walk_stop = Motion_state.new('walk_stop')
walk_stop:set_joint_access(0, 'all')
walk_stop:set_joint_access(1, 'lowerbody')
local dcm = walk_stop.dcm

-- default parameters
walk_stop.parameters = {
}

-- config parameters
if (Config.motion.walk_stop and Config.motion.walk_stop.parameters) then
  walk_stop:load_parameters(Config.motion.walk_stop.parameters)
end

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
