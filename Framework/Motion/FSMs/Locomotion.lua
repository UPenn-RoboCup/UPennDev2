---------------------------------------------------------
-- Locomotion State Machine 
---------------------------------------------------------

require('walk')
require('null_state')
require('Motion_fsm')

Locomotion = Motion_fsm.new(null_state)
Locomotion:add_state(null_state)
Locomotion:add_state(walk)

Locomotion:set_transition(null_state, 'walk', walk)

Locomotion:set_joint_access(0, 'upperbody')
return Locomotion
