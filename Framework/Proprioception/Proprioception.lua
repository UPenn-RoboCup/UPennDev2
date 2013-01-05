require('vector')
require('Config')
require('Kinematics')
require('cop_zmp')
require('pcm')

Proprioception = {}

function Proprioception.entry()
end

function Proprioception.update()
  -- update end-effector pose estimates
  local q_legs = dcm:get_joint_position('legs')
  local l_foot_frame, r_foot_frame = Kinematics.forward_pos_legs(q_legs)
  local l_foot_pose = l_foot_frame:get_pose6D()
  local r_foot_pose = r_foot_frame:get_pose6D()
  pcm:set_l_foot_pose(l_foot_pose)
  pcm:set_r_foot_pose(r_foot_pose)

  -- update center of pressure estimates
  local l_foot_cop, l_foot_pressure = cop_zmp.estimate_l_foot_cop()
  local r_foot_cop, r_foot_pressure = cop_zmp.estimate_r_foot_cop()
  local cop, pressure = cop_zmp.estimate_cop(l_foot_pose, r_foot_pose)
  pcm:set_l_foot_cop(l_foot_cop)
  pcm:set_l_foot_pressure(l_foot_pressure)
  pcm:set_r_foot_cop(r_foot_cop)
  pcm:set_r_foot_pressure(r_foot_pressure)
  pcm:set_cop(cop)
  pcm:set_cop_pressure(pressure)
end

function Proprioception.exit()
end

return Proprioception
