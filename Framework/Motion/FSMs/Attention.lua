require('look')
require('Motion_fsm')
require('Config')

---------------------------------------------------------
-- Attention State Machine 
---------------------------------------------------------

local joint = Config.joint

Attention = Motion_fsm.new(look)

Attention:set_joint_access(0, joint.all)
Attention:set_joint_access(1, joint.head)

return Attention
