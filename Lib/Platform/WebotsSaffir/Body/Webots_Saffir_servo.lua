module(..., package.seeall)

-- Servo ids 
---------------------------------------------------------------------

id = {
  [1] = 'l_hip_yaw',
  [2] = 'l_hip_inner',
  [3] = 'l_hip_outer',
  [4] = 'l_knee_pitch',
  [5] = 'l_ankle_inner',
  [6] = 'l_ankle_outer',
  [7] = 'r_hip_yaw',
  [8] = 'r_hip_inner',
  [9] = 'r_hip_outer',
  [10] = 'r_knee_pitch',
  [11] = 'r_ankle_inner',
  [12] = 'r_ankle_outer',
}

count = #id 
for k,v in pairs(id) do
  getfenv()[v] = k 
end

-- Servo parameters
---------------------------------------------------------------------

type = {
-- r = revolute, l = linear
  'r', 'l', 'l', 'l', 'l', 'l',
  'r', 'l', 'l', 'l', 'l', 'l',
}

position_offset = {
-- in radians or meters
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
}

force_offset = {
-- in newton meters or newtons
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
}

-- Motor parameters
---------------------------------------------------------------------

motor_position_direction = {
  1, -1, -1, -1, -1, -1,
  1, -1, -1, -1, -1, -1,
}

motor_force_direction = {
  1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1,
}

motor_position_offset = {
-- in motor units 
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
}

motor_force_offset = {
-- in motor units
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
}

motor_position_ratio = {
-- in radians or meters / motor unit
  1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1,
}

motor_force_ratio = {
-- in newton meters or newtons / motor unit
  1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1,
}

-- Servo groups 
---------------------------------------------------------------------

group = {
  l_leg = {
    l_hip_yaw,
    l_hip_inner,
    l_hip_outer,
    l_knee_pitch,
    l_ankle_inner,
    l_ankle_outer,
  },
  r_leg = {
    r_hip_yaw,
    r_hip_inner,
    r_hip_outer,
    r_knee_pitch,
    r_ankle_inner,
    r_ankle_outer,
  },
  legs = {
    l_hip_yaw,
    l_hip_inner,
    l_hip_outer,
    l_knee_pitch,
    l_ankle_inner,
    l_ankle_outer,
    r_hip_yaw,
    r_hip_inner,
    r_hip_outer,
    r_knee_pitch,
    r_ankle_inner,
    r_ankle_outer,
  },
  l_arm = {
  },
  r_arm = {
  },
  arms = {
  },
  head = {
  },
}

group.all = {}
for i = 1,count do
  group.all[i] = i
end

for k,v in pairs(group) do
  getfenv()[k] = v 
end
