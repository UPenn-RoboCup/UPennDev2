require('util')
require('vector')

---------------------------------------------------------------------------
-- Prioprioception Communication Module
---------------------------------------------------------------------------

pcm = {}

local shared_data = {}
shared_data.cog = vector.zeros(3)
shared_data.cop = vector.zeros(3)
shared_data.cop_pressure = vector.zeros(1) 
shared_data.l_foot_cop = vector.zeros(3)
shared_data.l_foot_cop_pressure = vector.zeros(1)
shared_data.r_foot_cop = vector.zeros(3)
shared_data.r_foot_cop_pressure = vector.zeros(1)
shared_data.l_foot_pose = vector.zeros(6)
shared_data.l_foot_twist = vector.zeros(6)
shared_data.l_foot_wrench = vector.zeros(6)
shared_data.r_foot_pose = vector.zeros(6)
shared_data.r_foot_twist = vector.zeros(6)
shared_data.r_foot_wrench = vector.zeros(6)
shared_data.l_hand_pose = vector.zeros(6)
shared_data.l_hand_twist = vector.zeros(6)
shared_data.l_hand_wrench = vector.zeros(6)
shared_data.r_hand_pose = vector.zeros(6)
shared_data.r_hand_twist = vector.zeros(6)
shared_data.r_hand_wrench = vector.zeros(6)
shared_data.torso_pose = vector.zeros(6)
shared_data.torso_twist = vector.zeros(6)
shared_data.torso_accel = vector.zeros(6)
shared_data.torso_orientation = vector.zeros(9)
shared_data.tipping_status = vector.zeros(3)
shared_data.falling_status = vector.zeros(3)

util.init_shm_module(pcm, 'pcm', shared_data)

return pcm
