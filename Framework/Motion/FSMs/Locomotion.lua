---------------------------------------------------------
-- Locomotion State Machine 
---------------------------------------------------------

require('walk')
require('nullState')
require('MotionFSM')

Locomotion = MotionFSM.new(nullState)
Locomotion:add_state(nullState)
Locomotion:add_state(walk)

Locomotion:set_transition(nullState, 'walk', walk)

return Locomotion
