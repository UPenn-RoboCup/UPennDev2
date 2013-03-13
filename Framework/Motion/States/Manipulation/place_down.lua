require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Place Down Controller
--------------------------------------------------------------------------

place_down = Motion_state.new('place_down')
place_down:set_joint_access(0, 'all')
place_down:set_joint_access(1, 'upperbody')
place_down:set_joint_access(0, 'head')
local dcm = place_down.dcm

-- default parameters
place_down.parameters = {
}

-- config parameters
place_down:load_parameters()

function place_down:entry()
end

function place_down:update()
end

function place_down:exit()
end

return place_down
