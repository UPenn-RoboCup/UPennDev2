require('twist')
require('wrench')
require('vector')
require('Transform')
require('shm_util')

---------------------------------------------------------------------------
-- Motion Communication Module
---------------------------------------------------------------------------

mcm = {}

local shared_data = {}
shared_data.desired_cog = vector.zeros(3)
shared_data.desired_cop = vector.zeros(3)
shared_data.desired_l_foot_pose = vector.zeros(6)
shared_data.desired_l_foot_twist = twist.zeros()
shared_data.desired_l_foot_wrench = wrench.zeros()
shared_data.desired_r_foot_pose = vector.zeros(6)
shared_data.desired_r_foot_twist = twist.zeros()
shared_data.desired_r_foot_wrench = wrench.zeros()
shared_data.desired_l_hand_pose = vector.zeros(6)
shared_data.desired_l_hand_twist = twist.zeros()
shared_data.desired_l_hand_wrench = wrench.zeros()
shared_data.desired_r_hand_pose = vector.zeros(6)
shared_data.desired_r_hand_twist = twist.zeros()
shared_data.desired_r_hand_wrench = wrench.zeros()
shared_data.desired_torso_orientation = Transform.eye()
shared_data.odometry = vector.zeros(3)

shm_util.init_shm_module(mcm, 'mcm', shared_data)

return mcm
