require('util')
require('vector')

---------------------------------------------------------------------------
-- Prioprioception Communication Module
---------------------------------------------------------------------------

pcm = {}

local shared_data = {}
shared_data.torso_rotation_euler = vector.zeros(3)
shared_data.torso_rotation_matrix = vector.zeros(9)
shared_data.l_foot_pose = vector.zeros(6)
shared_data.r_foot_pose = vector.zeros(6)
shared_data.l_foot_twist = vector.zeros(6)
shared_data.r_foot_twist = vector.zeros(6)
shared_data.l_hand_pose = vector.zeros(6)
shared_data.r_hand_pose = vector.zeros(6)
shared_data.l_hand_twist = vector.zeros(6)
shared_data.r_hand_twist = vector.zeros(6)
shared_data.contacts = vector.zeros(4)
shared_data.cog = vector.zeros(3)
shared_data.zmp = vector.zeros(3)
shared_data.cop = vector.zeros(3)
shared_data.cop_pressure = vector.zeros(1) 
shared_data.l_foot_cop = vector.zeros(3)
shared_data.l_foot_pressure = vector.zeros(1)
shared_data.r_foot_cop = vector.zeros(3)
shared_data.r_foot_pressure = vector.zeros(1)

util.init_shm_module(pcm, 'pcm', shared_data)

return pcm
