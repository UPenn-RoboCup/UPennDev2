module(..., package.seeall)

require('Transform')

mechanics = {}

mechanics.l_foot = {
  sole_dimensions = {0, 0},
  sole_transform = Transform.pose6D{0, 0, 0},
  heel_transform = Transform.pose6D{0, 0, 0},
  toe_transform = Transform.pose6D{0, 0, 0},
  force_torque_transform = Transform.pose6D{0, 0, 0}
}

mechanics.r_foot = {
  sole_dimensions = {0, 0},
  sole_transform = Transform.pose6D{0, 0, 0},
  heel_transform = Transform.pose6D{0, 0, 0},
  toe_transform = Transform.pose6D{0, 0, 0},
  force_torque_transform = Transform.pose6D{0, 0, 0}
}

