dofile('include.lua')

require('dcm')

dcm:set_joint_position_p_gain(1, 'all')
dcm:set_joint_position_i_gain(0, 'all')
dcm:set_joint_position_d_gain(0, 'all')
dcm:set_joint_velocity_p_gain(0, 'all')
dcm:set_joint_force(0, 'all')
dcm:set_joint_position(dcm:get_joint_position_sensor())
dcm:set_joint_velocity(0, 'all')
dcm:set_joint_enable(0, 'all')
