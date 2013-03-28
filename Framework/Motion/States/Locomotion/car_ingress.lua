require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Car Ingress Controller
--------------------------------------------------------------------------

car_ingress = Motion_state.new('car_ingress')
car_ingress:set_joint_access(0, 'all')
car_ingress:set_joint_access(1, 'lowerbody')
local dcm = car_ingress.dcm

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
