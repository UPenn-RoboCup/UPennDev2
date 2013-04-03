require('dcm')
require('pcm')
require('twist')
require('vector')
require('Config')
require('Transform')
require('Kinematics')

--------------------------------------------------------------------------------
-- fk_sensor : estimates end-effector poses and twists from forward kinematics
--------------------------------------------------------------------------------

fk_sensor = {}

local joint = Config.joint

function fk_sensor.entry()
end

function fk_sensor.update()
  local q_waist = dcm:get_joint_position_sensor(joint.waist)
  local q_legs = dcm:get_joint_position_sensor(joint.legs)
  local q_arms = dcm:get_joint_position_sensor(joint.arms)
  local qdot_waist = dcm:get_joint_velocity_sensor(joint.waist)
  local qdot_legs = dcm:get_joint_velocity_sensor(joint.legs)
  local qdot_arms = dcm:get_joint_velocity_sensor(joint.arms)
  local torso_frame = pcm:get_torso_rotation()
  local torso_twist = pcm:get_torso_twist()

  local chest_frame, chest_twist = 
    Kinematics.forward_vel_waist(q_waist, qdot_waist, torso_frame, torso_twist)
  local l_foot_frame, r_foot_frame, l_foot_twist, r_foot_twist =
    Kinematics.forward_vel_legs(q_legs, qdot_legs, torso_frame, torso_twist)
  local l_hand_frame, r_hand_frame, l_hand_twist, r_hand_twist =
    Kinematics.forward_vel_arms(q_arms, qdot_arms, chest_frame, chest_twist)

  pcm:set_l_foot_pose(l_foot_frame:get_pose())
  pcm:set_r_foot_pose(r_foot_frame:get_pose())
  pcm:set_l_hand_pose(l_hand_frame:get_pose())
  pcm:set_r_hand_pose(r_hand_frame:get_pose())
  pcm:set_l_foot_twist(l_foot_twist)
  pcm:set_r_foot_twist(r_foot_twist)
  pcm:set_l_hand_twist(l_hand_twist)
  pcm:set_r_hand_twist(r_hand_twist)
end

function fk_sensor.exit()
end

return fk_sensor
