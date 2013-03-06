require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Fall Controller
--------------------------------------------------------------------------

fall = Motion_state.new('fall')
fall:set_joint_access(0, 'all')
fall:set_joint_access(1, 'lowerbody')
local dcm = fall.dcm

-- default parameters
fall.parameters = {
}

-- config parameters
fall:load_parameters()

function fall:entry()
end

function fall:update()
end

function fall:exit()
end

return fall
