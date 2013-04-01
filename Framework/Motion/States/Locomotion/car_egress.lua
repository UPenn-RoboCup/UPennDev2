require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Car Egress Controller
--------------------------------------------------------------------------

car_egress = Motion_state.new('car_egress')

local dcm = car_egress.dcm
local ahrs = Config.ahrs
local joint = Config.joint
car_egress:set_joint_access(0, joint.all)
car_egress:set_joint_access(1, joint.lowerbody)

-- default parameters
car_egress.parameters = {
}

-- config parameters
car_egress:load_parameters()

function car_egress:entry()
end

function car_egress:update()
end

function car_egress:exit()
end

return car_egress
