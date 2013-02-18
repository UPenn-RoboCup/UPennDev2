require('dcm')
require('pcm')
require('vector')
require('Config')
require('Kinematics')
require('Transform')

--------------------------------------------------------------------------------
-- fk_sensor : estimates end-effector poses and twists from forward kinematics
--------------------------------------------------------------------------------

fk_sensor = {}

function fk_sensor.entry()
end

function fk_sensor.update()
  local q_legs = dcm:get_joint_position_sensor('legs')
  local qdot_legs = dcm:get_joint_velocity_sensor('legs')

  -- TODO use torso_orientation instead of torso_pose
  local torso_twist = pcm:get_torso_twist()
  local torso_pose = pcm:get_torso_pose()
  local torso_frame = Transform.pose6D(torso_pose)

  local l_foot_frame, r_foot_frame, l_foot_twist, r_foot_twist =
    Kinematics.forward_vel_legs(q_legs, qdot_legs, torso_frame, torso_twist)
  local l_foot_pose = l_foot_frame:get_pose6D()
  local r_foot_pose = r_foot_frame:get_pose6D()

  pcm:set_l_foot_pose(l_foot_pose)
  pcm:set_r_foot_pose(r_foot_pose)
  pcm:set_l_foot_twist(l_foot_twist)
  pcm:set_r_foot_twist(r_foot_twist)

  -- TODO implement upperbody forward kinematics
end

function fk_sensor.exit()
end

return reference_sensor
