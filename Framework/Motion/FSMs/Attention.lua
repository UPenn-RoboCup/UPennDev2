require('look')
require('Motion_fsm')

---------------------------------------------------------
-- Attention State Machine 
---------------------------------------------------------

Attention = Motion_fsm.new(look)

Attention:set_joint_access(0, 'all')
Attention:set_joint_access(1, 'head')

return Attention
