require('Motion_state')

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

function car_egress:entry()
end

function car_egress:update()
end

function car_egress:exit()
end

return car_egress
