require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Track Controller
--------------------------------------------------------------------------

track = Motion_state.new('track')
track:set_joint_access(0, 'all')
track:set_joint_access(1, 'head')
local dcm = track.dcm

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
