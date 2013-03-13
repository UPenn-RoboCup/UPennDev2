require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Pick Up Controller
--------------------------------------------------------------------------

pick_up = Motion_state.new('pick_up')
pick_up:set_joint_access(0, 'all')
pick_up:set_joint_access(1, 'upperbody')
pick_up:set_joint_access(0, 'head')
local dcm = pick_up.dcm

-- default parameters
pick_up.parameters = {
}

-- config parameters
pick_up:load_parameters()

function pick_up:entry()
end

function pick_up:update()
end

function pick_up:exit()
end

return pick_up
