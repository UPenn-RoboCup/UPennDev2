require('vector')
require('Config')
require('shm_util')

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
shared_data.l_foot_twist = twist.zeros()
shared_data.l_foot_wrench = wrench.zeros()
shared_data.r_foot_pose = vector.zeros(6)
shared_data.r_foot_twist = twist.zeros()
shared_data.r_foot_wrench = wrench.zeros()
shared_data.l_hand_pose = vector.zeros(6)
shared_data.l_hand_twist = twist.zeros()
shared_data.l_hand_wrench = wrench.zeros()
shared_data.r_hand_pose = vector.zeros(6)
shared_data.r_hand_twist = twist.zeros()
shared_data.r_hand_wrench = wrench.zeros()
shared_data.torso_pose = vector.zeros(6)
shared_data.torso_twist = twist.zeros()
shared_data.torso_rotation = Transform.eye()
shared_data.joint_force = vector.zeros(#Config.joint.id)
shared_data.joint_position = vector.zeros(#Config.joint.id)
shared_data.joint_velocity = vector.zeros(#Config.joint.id)
shared_data.contact_status = vector.zeros(4)
shared_data.tipping_status = vector.zeros(1)
shared_data.falling_status = vector.zeros(1)

shm_util.init_shm_module(pcm, 'pcm', shared_data)

return pcm
