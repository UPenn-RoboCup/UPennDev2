require('Motion_state')
require('Config')

null_state = Motion_state.new('null_state')

local joint = Config.joint
null_state:set_joint_access(0, joint.all)

return null_state
