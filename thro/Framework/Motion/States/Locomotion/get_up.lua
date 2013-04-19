require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Get Up Controller
--------------------------------------------------------------------------

get_up = Motion_state.new('get_up')

local dcm = get_up.dcm
local ahrs = Config.ahrs
local joint = Config.joint
get_up:set_joint_access(0, joint.all)
get_up:set_joint_access(1, joint.lowerbody)

-- default parameters
get_up.parameters = {
}

-- config parameters
get_up:load_parameters()

function get_up:entry()
end

function get_up:update()
end

function get_up:exit()
end

return get_up
