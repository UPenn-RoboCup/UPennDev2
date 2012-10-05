dofile('include.lua')

require('dcm')

dcm:set_joint_stiffness(1, 'all')
dcm:set_joint_damping(0, 'all')
dcm:set_joint_force(0, 'all')
dcm:set_joint_position(dcm:get_joint_position_sensor())
dcm:set_joint_velocity(0, 'all')
dcm:set_joint_enable(0, 'all')
