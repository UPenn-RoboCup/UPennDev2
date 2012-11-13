require('util')
require('vector')

---------------------------------------------------------------------------
-- Motion Communication Module
---------------------------------------------------------------------------

mcm = {}

local shared_data = {}
shared_data.com = vector.zeros(3)
shared_data.cop = vector.zeros(3)
shared_data.cop_magnitude = vector.zeros(1) 
shared_data.l_foot_cop = vector.zeros(1)
shared_data.l_foot_cop_magnitude = vector.zeros(1)
shared_data.r_foot_cop = vector.zeros(1)
shared_data.l_foot_cop_magnitude = vector.zeros(1)
shared_data.l_foot_position = vector.zeros(6)
shared_data.r_foot_position = vector.zeros(6)
shared_data.l_hand_position = vector.zeros(6)
shared_data.r_hand_position = vector.zeros(6)
shared_data.torso_rotation_euler = vector.zeros(3)
shared_data.torso_rotation_matrix = vector.zeros(9)
shared_data.contacts = vector.zeros(4)

util.init_shm_module(mcm, 'mcm', shared_data)

return mcm
