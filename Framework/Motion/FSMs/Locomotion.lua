---------------------------------------------------------
-- Locomotion State Machine 
---------------------------------------------------------

require('walk')
require('MotionFSM')

Locomotion = MotionFSM.new(walk)

Locomotion:set_transition(walk, 'done', walk)

return Locomotion
