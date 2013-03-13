require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Open Door Controller
--------------------------------------------------------------------------

open_door = Motion_state.new('open_door')
open_door:set_joint_access(0, 'all')
open_door:set_joint_access(1, 'upperbody')
open_door:set_joint_access(0, 'head')
local dcm = open_door.dcm

-- default parameters
open_door.parameters = {
}

-- config parameters
open_door:load_parameters()

function open_door:entry()
end

function open_door:update()
end

function open_door:exit()
end

return open_door
