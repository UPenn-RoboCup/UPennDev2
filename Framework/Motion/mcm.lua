require('util')
require('vector')

---------------------------------------------------------------------------
-- Motion Communication Module
---------------------------------------------------------------------------

mcm = {}

local shared_data = {}
shared_data.odometry = vector.zeros(3)
shared_data.desired_cop = vector.zeros(3)

util.init_shm_module(mcm, 'mcm', shared_data)

return mcm
