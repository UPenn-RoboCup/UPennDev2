require('Motion_state')

null_state = Motion_state.new('null_state')

null_state:set_joint_access(0, 'all')

return null_state
