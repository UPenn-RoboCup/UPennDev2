require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Track Controller
--------------------------------------------------------------------------

track = Motion_state.new('track')

local dcm = track.dcm
local joint = Config.joint
track:set_joint_access(0, joint.all)
track:set_joint_access(1, joint.head)

-- default parameters
track.parameters = {
}

-- config parameters
track:load_parameters()

function track:entry()
end

function track:update()
end

function track:exit()
end

return track
