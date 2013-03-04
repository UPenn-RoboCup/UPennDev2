require('Motion_state')
require('Config')

--------------------------------------------------------------------------
-- Car Egress Controller
--------------------------------------------------------------------------

car_egress = Motion_state.new('car_egress')
car_egress:set_joint_access(0, 'all')
car_egress:set_joint_access(1, 'lowerbody')
local dcm = car_egress.dcm

-- default parameters
car_egress.parameters = {
}

-- config parameters
if (Config.motion.car_egress and Config.motion.car_egress.parameters) then
  car_egress:load_parameters(Config.motion.car_egress.parameters)
end

function car_egress:entry()
end

function car_egress:update()
end

function car_egress:exit()
end

return car_egress
