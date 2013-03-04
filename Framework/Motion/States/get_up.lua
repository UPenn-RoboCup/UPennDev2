require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Get Up Controller
--------------------------------------------------------------------------

get_up = Motion_state.new('get_up')
get_up:set_joint_access(0, 'all')
get_up:set_joint_access(1, 'lowerbody')
local dcm = get_up.dcm

-- default parameters
get_up.parameters = {
}

-- config parameters
if (Config.motion.get_up and Config.motion.get_up.parameters) then
  get_up:load_parameters(Config.motion.get_up.parameters)
end

function get_up:entry()
end

function get_up:update()
end

function get_up:exit()
end

return get_up
