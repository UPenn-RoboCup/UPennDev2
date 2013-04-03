require('Config')
require('Motion_fsm')
require('idle')
require('stand')
require('walk_stop')
require('walk')

--------------------------------------------------------------------------------
-- Locomotion State Machine 
--------------------------------------------------------------------------------

local joint = Config.joint

Locomotion = Motion_fsm.new(idle)
Locomotion:add_state(stand)
Locomotion:add_state(walk)
Locomotion:add_state(walk_stop)

Locomotion:set_joint_access(0, joint.all)
Locomotion:set_joint_access(1, joint.lowerbody)

Locomotion:set_transition(idle, 'stand', stand)
Locomotion:set_transition(stand, 'idle', idle)
Locomotion:set_transition(stand, 'walk', walk)
Locomotion:set_transition(walk, 'unsafe', stand)
Locomotion:set_transition(walk, 'done', stand)
Locomotion:set_transition(walk, 'stand', walk_stop)
Locomotion:set_transition(walk_stop, 'done', stand)

return Locomotion
