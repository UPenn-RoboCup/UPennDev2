require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Car Ingress Controller
--------------------------------------------------------------------------

car_ingress = Motion_state.new('car_ingress')

local dcm = car_ingress.dcm
local ahrs = Config.ahrs
local joint = Config.joint
car_ingress:set_joint_access(0, joint.all)
car_ingress:set_joint_access(1, joint.lowerbody)

-- default parameters
car_ingress.parameters = {
}

-- config parameters
car_ingress:load_parameters()

function car_ingress:entry()
end

function car_ingress:update()
end

function car_ingress:exit()
end

return car_ingress
