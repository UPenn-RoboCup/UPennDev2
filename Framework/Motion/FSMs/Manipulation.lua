require('Config')
require('Motion_fsm')
require('locomotion_slave')
require('teleop')
require('wield_tool')
require('place_down')
require('turn_wheel')
require('open_door')
require('pick_up')
require('carry')

--------------------------------------------------------------------------------
-- Manipulation State Machine 
--------------------------------------------------------------------------------

local joint = Config.joint

Manipulation = Motion_fsm.new(locomotion_slave)
Manipulation:add_state(teleop)
--[[
Manipulation:add_state(wield_tool)
Manipulation:add_state(place_down)
Manipulation:add_state(turn_wheel)
Manipulation:add_state(open_door)
Manipulation:add_state(pick_up)
Manipulation:add_state(carry)
--]]

Manipulation:set_joint_access(0, joint.all)
Manipulation:set_joint_access(1, joint.upperbody)
Manipulation:set_joint_access(0, joint.head)

Manipulation:set_transition(locomotion_slave, 'teleop', teleop)
Manipulation:set_transition(teleop, 'locomotion', locomotion_slave)
Manipulation:set_transition(teleop, 'fall', locomotion_slave)
--[[
Manipulation:set_transition(teleop, 'wield_tool', wield_tool)
Manipulation:set_transition(teleop, 'place_down', place_down)
Manipulation:set_transition(teleop, 'turn_wheel', turn_wheel)
Manipulation:set_transition(teleop, 'open_door', open_door)
Manipulation:set_transition(teleop, 'pick_up', pick_up)
Manipulation:set_transition(wield_tool, 'teleop', teleop)
Manipulation:set_transition(place_down, 'teleop', teleop)
Manipulation:set_transition(place_down, 'done', teleop)
Manipulation:set_transition(turn_wheel, 'teleop', teleop)
Manipulation:set_transition(open_door, 'teleop', teleop)
Manipulation:set_transition(open_door, 'done', teleop)
Manipulation:set_transition(pick_up, 'teleop', teleop)
Manipulation:set_transition(pick_up, 'done', teleop)
--]]

return Manipulation
