dofile('include.lua')

require('acm')
require('scm')

acm:set_joint_stiffness(0, 'all')
acm:set_joint_damping(0, 'all')
acm:set_joint_force(0, 'all')
acm:set_joint_position(scm:get_joint_position())
acm:set_joint_enable(0, 'all')
