require('util')
require('vector')

---------------------------------------------------------------------------
-- Motion Communication Module
---------------------------------------------------------------------------

mcm = {}

local shared_data = {}
shared_data.desired_cog = vector.zeros(3)
shared_data.desired_cop = vector.zeros(3)
shared_data.desired_l_foot_pose = vector.zeros(6)
shared_data.desired_l_foot_twist = vector.zeros(6)
shared_data.desired_l_foot_wrench = vector.zeros(6)
shared_data.desired_r_foot_pose = vector.zeros(6)
shared_data.desired_r_foot_twist = vector.zeros(6)
shared_data.desired_r_foot_wrench = vector.zeros(6)
shared_data.desired_l_hand_pose = vector.zeros(6)
shared_data.desired_l_hand_twist = vector.zeros(6)
shared_data.desired_l_hand_wrench = vector.zeros(6)
shared_data.desired_r_hand_pose = vector.zeros(6)
shared_data.desired_r_hand_twist = vector.zeros(6)
shared_data.desired_r_hand_wrench = vector.zeros(6)
shared_data.desired_torso_orientation = vector.zeros(9)
shared_data.odometry = vector.zeros(3)

util.init_shm_module(mcm, 'mcm', shared_data)

return mcm
