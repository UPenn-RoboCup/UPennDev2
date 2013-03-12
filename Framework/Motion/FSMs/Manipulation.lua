require('locomotion_slave')
require('Motion_fsm')

---------------------------------------------------------
-- Manipulation State Machine 
---------------------------------------------------------

Manipulation = Motion_fsm.new(locomotion_slave)

Manipulation:set_joint_access(0, 'all')
Manipulation:set_joint_access(1, 'upperbody')
Manipulation:set_joint_access(0, 'head')

return Manipulation
