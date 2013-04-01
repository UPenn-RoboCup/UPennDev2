require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Open Door Controller
--------------------------------------------------------------------------

open_door = Motion_state.new('open_door')

local dcm = open_door.dcm
local joint = Config.joint
open_door:set_joint_access(0, joint.all)
open_door:set_joint_access(1, joint.upperbody)
open_door:set_joint_access(0, joint.head)

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
