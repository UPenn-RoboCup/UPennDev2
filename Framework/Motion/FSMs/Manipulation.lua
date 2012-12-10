---------------------------------------------------------
-- Manipulation State Machine 
---------------------------------------------------------

require('nullState')
require('MotionFSM')

Manipulation = MotionFSM.new(nullState)

Manipulation:set_joint_access(0, 'legs')
return Manipulation
