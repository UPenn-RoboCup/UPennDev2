require('pcm')
require('Kinematics')
require('cop_sensor')
require('contact_sensor')

Proprioception = {}

function Proprioception.entry()
  contact_sensor.entry()
  cop_sensor.entry()
end

function Proprioception.update()
  -- update forward kinematics 
  local q_legs = dcm:get_joint_position_sensor('legs')
  local qdot_legs = dcm:get_joint_velocity_sensor('legs')

  local l_foot_frame, r_foot_frame, l_foot_twist, r_foot_twist =
    Kinematics.forward_vel_legs(q_legs, qdot_legs)
  local l_foot_pose = l_foot_frame:get_pose6D()
  local r_foot_pose = r_foot_frame:get_pose6D()

  pcm:set_l_foot_pose(l_foot_pose)
  pcm:set_r_foot_pose(r_foot_pose)
  pcm:set_l_foot_twist(l_foot_twist)
  pcm:set_r_foot_twist(r_foot_twist)

  -- update virtual sensors 
  contact_sensor.update()
  cop_sensor.update()
end

function Proprioception.exit()
  contact_sensor.exit()
  cop_sensor.exit()
end

return Proprioception
