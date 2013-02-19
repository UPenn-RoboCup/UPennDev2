---------------------------------------------------------
-- Manipulation State Machine 
---------------------------------------------------------

require('null_state')
require('MotionFSM')

Manipulation = MotionFSM.new(null_state)

Manipulation:set_joint_access(0, 'legs')
return Manipulation
