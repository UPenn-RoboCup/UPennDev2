---------------------------------------------------------
-- Manipulation State Machine 
---------------------------------------------------------

require('null_state')
require('Motion_fsm')

Manipulation = Motion_fsm.new(null_state)

Manipulation:set_joint_access(0, 'legs')
return Manipulation
