module(..., package.seeall)

require('servo')

-- Joint ids 
---------------------------------------------------------------------

id = {
  [1] = 'l_hip_yaw',
  [2] = 'l_hip_roll',
  [3] = 'l_hip_pitch',
  [4] = 'l_knee_pitch',
  [5] = 'l_ankle_pitch',
  [6] = 'l_ankle_roll',
  [7] = 'r_hip_yaw',
  [8] = 'r_hip_roll',
  [9] = 'r_hip_pitch',
  [10] = 'r_knee_pitch',
  [11] = 'r_ankle_pitch',
  [12] = 'r_ankle_roll',
}

count = #id 
for k,v in pairs(id) do
  getfenv()[v] = k 
end

-- Servo mapping
---------------------------------------------------------------------

servo_map = {
  [l_hip_yaw] = {servo.l_hip_yaw},
  [l_hip_roll] = {servo.l_hip_inner, servo.l_hip_outer},
  [l_hip_pitch] = {servo.l_hip_inner, servo.l_hip_outer},
  [l_knee_pitch] = {servo.l_knee_pitch},
  [l_ankle_pitch] = {servo.l_ankle_inner, servo.l_ankle_outer},
  [l_ankle_roll] = {servo.l_ankle_inner, servo.l_ankle_outer},
  [r_hip_yaw] = {servo.r_hip_yaw},
  [r_hip_roll] = {servo.r_hip_yaw, servo.r_hip_inner, servo.r_hip_outer},
  [r_hip_pitch] = {servo.r_hip_yaw, servo.r_hip_inner, servo.r_hip_outer},
  [r_knee_pitch] = {servo.r_knee_pitch},
  [r_ankle_pitch] = {servo.r_ankle_inner, servo.r_ankle_outer},
  [r_ankle_roll] = {servo.r_ankle_inner, servo.r_ankle_outer},
}

coupled = {}
for i,map_A in pairs(servo_map) do
  coupled[i] = {}
  for j,map_B in pairs(servo_map) do
    for k,servo_A in pairs(map_A) do
      for h,servo_B in pairs(map_B) do
        if (servo_A == servo_B) then
          coupled[i][j] = true
        end
      end
    end
  end
end

-- Joint parameters
---------------------------------------------------------------------

angle_offset = {
-- in radians
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
}

torque_offset = {
-- in newton meters
  0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0,
}

-- Joint groups
---------------------------------------------------------------------

group = {
  l_leg = {
    l_hip_yaw,
    l_hip_roll,
    l_hip_pitch,
    l_knee_pitch,
    l_ankle_pitch,
    l_ankle_roll,
  },
  r_leg = {
    r_hip_yaw,
    r_hip_roll,
    r_hip_pitch,
    r_knee_pitch,
    r_ankle_pitch,
    r_ankle_roll,
  },
  legs = {
    l_hip_yaw,
    l_hip_roll,
    l_hip_pitch,
    l_knee_pitch,
    l_ankle_pitch,
    l_ankle_roll,
    r_hip_yaw,
    r_hip_roll,
    r_hip_pitch,
    r_knee_pitch,
    r_ankle_pitch,
    r_ankle_roll,
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
