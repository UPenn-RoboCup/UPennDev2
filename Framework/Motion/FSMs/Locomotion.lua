---------------------------------------------------------
-- Locomotion State Machine 
---------------------------------------------------------

require('walk')
require('null_state')
require('MotionFSM')

Locomotion = MotionFSM.new(null_state)
Locomotion:add_state(null_state)
Locomotion:add_state(walk)

Locomotion:set_transition(null_state, 'walk', walk)

Locomotion:set_joint_access(0, 'upperbody')
return Locomotion
