require('idle')
require('stand')
require('walk')
require('walk_stop')
require('Motion_fsm')

---------------------------------------------------------
-- Locomotion State Machine 
---------------------------------------------------------

Locomotion = Motion_fsm.new(idle)
Locomotion:add_state(stand)
Locomotion:add_state(walk)
Locomotion:add_state(walk_stop)

Locomotion:set_joint_access(0, 'all')
Locomotion:set_joint_access(1, 'lowerbody')

Locomotion:set_transition(idle, 'stand', stand)
Locomotion:set_transition(stand, 'idle', idle)
Locomotion:set_transition(stand, 'walk', walk)
Locomotion:set_transition(walk, 'unsafe', stand)
Locomotion:set_transition(walk, 'done', stand)
Locomotion:set_transition(walk, 'stand', walk_stop)
Locomotion:set_transition(walk_stop, 'done', stand)

return Locomotion
