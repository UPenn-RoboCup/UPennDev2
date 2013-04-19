module(..., package.seeall)

devices = {
  'joint',
  'motor',
  'force_torque',
  'ahrs',
  'battery',
}

-- joint config
---------------------------------------------------------------------------

joint = {}

joint.id = { 
  -- device ids
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
  [13] = 'waist_yaw',
  [14] = 'waist_pitch',
  [15] = 'l_shoulder_pitch',
  [16] = 'l_shoulder_roll',
  [17] = 'l_shoulder_yaw',
  [18] = 'l_elbow_pitch',
  [19] = 'l_wrist_yaw',
  [20] = 'l_wrist_roll',
  [21] = 'r_shoulder_pitch',
  [22] = 'r_shoulder_roll',
  [23] = 'r_shoulder_yaw',
  [24] = 'r_elbow_pitch',
  [25] = 'r_wrist_yaw',
  [26] = 'r_wrist_roll',
  [27] = 'head_yaw',
  [28] = 'head_pitch',
  [29] = 'l_gripper',
  [30] = 'r_gripper',
  [31] = 'l_finger',
  [32] = 'r_finger',

  -- group ids
  l_hip = {
    'l_hip_yaw',
    'l_hip_roll',
    'l_hip_pitch',
  },
  r_hip = {
    'r_hip_yaw',
    'r_hip_roll',
    'r_hip_pitch',
  },
  hips = {
    'l_hip_yaw',
    'l_hip_roll',
    'l_hip_pitch',
    'r_hip_yaw',
    'r_hip_roll',
    'r_hip_pitch',
  },
  l_knee = {
    'l_knee_pitch',
  },
  r_knee = {
    'r_knee_pitch',
  },
  knees = {
    'l_knee_pitch',
    'r_knee_pitch',
  },
  l_ankle = {
    'l_ankle_pitch',
    'l_ankle_roll',
  },
  r_ankle = {
    'r_ankle_pitch',
    'r_ankle_roll',
  },
  ankles = {
    'l_ankle_pitch',
    'l_ankle_roll',
    'r_ankle_pitch',
    'r_ankle_roll',
  },
  l_leg = {
    'l_hip_yaw',
    'l_hip_roll',
    'l_hip_pitch',
    'l_knee_pitch',
    'l_ankle_pitch',
    'l_ankle_roll',
  },
  r_leg = {
    'r_hip_yaw',
    'r_hip_roll',
    'r_hip_pitch',
    'r_knee_pitch',
    'r_ankle_pitch',
    'r_ankle_roll',
  },
  legs = {
    'l_hip_yaw',
    'l_hip_roll',
    'l_hip_pitch',
    'l_knee_pitch',
    'l_ankle_pitch',
    'l_ankle_roll',
    'r_hip_yaw',
    'r_hip_roll',
    'r_hip_pitch',
    'r_knee_pitch',
    'r_ankle_pitch',
    'r_ankle_roll',
  },
  waist = {
    'waist_yaw',
    'waist_pitch',
  },
  l_arm = {
    'l_shoulder_pitch',
    'l_shoulder_roll',
    'l_shoulder_yaw',
    'l_elbow_pitch',
    'l_wrist_yaw',
    'l_wrist_roll',
  },
  r_arm = {
    'r_shoulder_pitch',
    'r_shoulder_roll',
    'r_shoulder_yaw',
    'r_elbow_pitch',
    'r_wrist_yaw',
    'r_wrist_roll',
  },
  arms = {
    'l_shoulder_pitch',
    'l_shoulder_roll',
    'l_shoulder_yaw',
    'l_elbow_pitch',
    'l_wrist_yaw',
    'l_wrist_roll',
    'r_shoulder_pitch',
    'r_shoulder_roll',
    'r_shoulder_yaw',
    'r_elbow_pitch',
    'r_wrist_yaw',
    'r_wrist_roll',
  },
  l_hand = {
    'l_gripper',
    'l_finger'
  },
  r_hand = {
    'r_gripper',
    'r_finger'
  },
  hands = {
    'l_gripper',
    'r_gripper',
    'l_finger',
    'r_finger'
  },
  head = {
    'head_yaw',
    'head_pitch',
  },
  lowerbody = {
    'l_hip_yaw',
    'l_hip_roll',
    'l_hip_pitch',
    'l_knee_pitch',
    'l_ankle_pitch',
    'l_ankle_roll',
    'r_hip_yaw',
    'r_hip_roll',
    'r_hip_pitch',
    'r_knee_pitch',
    'r_ankle_pitch',
    'r_ankle_roll',
  },
  upperbody = {
    'waist_yaw',
    'waist_pitch',
    'l_shoulder_pitch',
    'l_shoulder_roll',
    'l_shoulder_yaw',
    'l_elbow_pitch',
    'l_wrist_yaw',
    'l_wrist_roll',
    'r_shoulder_pitch',
    'r_shoulder_roll',
    'r_shoulder_yaw',
    'r_elbow_pitch',
    'r_wrist_yaw',
    'r_wrist_roll',
    'head_yaw',
    'head_pitch',
    'l_gripper',
    'r_gripper',
    'l_finger',
    'r_finger',
  },
}

-- motor config
---------------------------------------------------------------------------

motor = {}

motor.id = { 
  -- device ids
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
  [13] = 'waist_yaw',
  [14] = 'waist_pitch',
  [15] = 'l_shoulder_pitch',
  [16] = 'l_shoulder_roll',
  [17] = 'l_shoulder_yaw',
  [18] = 'l_elbow_pitch',
  [19] = 'l_wrist_yaw',
  [20] = 'l_wrist_roll',
  [21] = 'r_shoulder_pitch',
  [22] = 'r_shoulder_roll',
  [23] = 'r_shoulder_yaw',
  [24] = 'r_elbow_pitch',
  [25] = 'r_wrist_yaw',
  [26] = 'r_wrist_roll',
  [27] = 'head_yaw',
  [28] = 'head_pitch',
  [29] = 'l_gripper',
  [30] = 'r_gripper',
  [31] = 'l_finger',
  [32] = 'r_finger',

  -- group ids
  l_hip = {
    'l_hip_yaw',
    'l_hip_roll',
    'l_hip_pitch',
  },
  r_hip = {
    'r_hip_yaw',
    'r_hip_roll',
    'r_hip_pitch',
  },
  hips = {
    'l_hip_yaw',
    'l_hip_roll',
    'l_hip_pitch',
    'r_hip_yaw',
    'r_hip_roll',
    'r_hip_pitch',
  },
  l_knee = {
    'l_knee_pitch',
  },
  r_knee = {
    'r_knee_pitch',
  },
  knees = {
    'l_knee_pitch',
    'r_knee_pitch',
  },
  l_ankle = {
    'l_ankle_pitch',
    'l_ankle_roll',
  },
  r_ankle = {
    'r_ankle_pitch',
    'r_ankle_roll',
  },
  ankles = {
    'l_ankle_pitch',
    'l_ankle_roll',
    'r_ankle_pitch',
    'r_ankle_roll',
  },
  l_leg = {
    'l_hip_yaw',
    'l_hip_roll',
    'l_hip_pitch',
    'l_knee_pitch',
    'l_ankle_pitch',
    'l_ankle_roll',
  },
  r_leg = {
    'r_hip_yaw',
    'r_hip_roll',
    'r_hip_pitch',
    'r_knee_pitch',
    'r_ankle_pitch',
    'r_ankle_roll',
  },
  legs = {
    'l_hip_yaw',
    'l_hip_roll',
    'l_hip_pitch',
    'l_knee_pitch',
    'l_ankle_pitch',
    'l_ankle_roll',
    'r_hip_yaw',
    'r_hip_roll',
    'r_hip_pitch',
    'r_knee_pitch',
    'r_ankle_pitch',
    'r_ankle_roll',
  },
  waist = {
    'waist_yaw',
    'waist_pitch',
  },
  l_arm = {
    'l_shoulder_pitch',
    'l_shoulder_roll',
    'l_shoulder_yaw',
    'l_elbow_pitch',
    'l_wrist_yaw',
    'l_wrist_roll',
  },
  r_arm = {
    'r_shoulder_pitch',
    'r_shoulder_roll',
    'r_shoulder_yaw',
    'r_elbow_pitch',
    'r_wrist_yaw',
    'r_wrist_roll',
  },
  arms = {
    'l_shoulder_pitch',
    'l_shoulder_roll',
    'l_shoulder_yaw',
    'l_elbow_pitch',
    'l_wrist_yaw',
    'l_wrist_roll',
    'r_shoulder_pitch',
    'r_shoulder_roll',
    'r_shoulder_yaw',
    'r_elbow_pitch',
    'r_wrist_yaw',
    'r_wrist_roll',
  },
  l_hand = {
    'l_gripper',
    'l_finger'
  },
  r_hand = {
    'r_gripper',
    'r_finger'
  },
  hands = {
    'l_gripper',
    'r_gripper',
    'l_finger',
    'r_finger'
  },
  head = {
    'head_yaw',
    'head_pitch',
  },
  lowerbody = {
    'l_hip_yaw',
    'l_hip_roll',
    'l_hip_pitch',
    'l_knee_pitch',
    'l_ankle_pitch',
    'l_ankle_roll',
    'r_hip_yaw',
    'r_hip_roll',
    'r_hip_pitch',
    'r_knee_pitch',
    'r_ankle_pitch',
    'r_ankle_roll',
  },
  upperbody = {
    'waist_yaw',
    'waist_pitch',
    'l_shoulder_pitch',
    'l_shoulder_roll',
    'l_shoulder_yaw',
    'l_elbow_pitch',
    'l_wrist_yaw',
    'l_wrist_roll',
    'r_shoulder_pitch',
    'r_shoulder_roll',
    'r_shoulder_yaw',
    'r_elbow_pitch',
    'r_wrist_yaw',
    'r_wrist_roll',
    'head_yaw',
    'head_pitch',
    'l_gripper',
    'r_gripper',
    'l_finger',
    'r_finger',
  },
}

-- force_torque config
---------------------------------------------------------------------------

force_torque = {}

force_torque.id = {
  -- device ids
  [1] = 'l_foot_force_x',
  [2] = 'l_foot_force_y',
  [3] = 'l_foot_force_z',
  [4] = 'l_foot_torque_x',
  [5] = 'l_foot_torque_y',
  [6] = 'l_foot_torque_z',
  [7] = 'r_foot_force_x',
  [8] = 'r_foot_force_y',
  [9] = 'r_foot_force_z',
  [10] = 'r_foot_torque_x',
  [11] = 'r_foot_torque_y',
  [12] = 'r_foot_torque_z',
  [13] = 'l_hand_force_x',
  [14] = 'l_hand_force_y',
  [15] = 'l_hand_force_z',
  [16] = 'l_hand_torque_x',
  [17] = 'l_hand_torque_y',
  [18] = 'l_hand_torque_z',
  [19] = 'r_hand_force_x',
  [20] = 'r_hand_force_y',
  [21] = 'r_hand_force_z',
  [22] = 'r_hand_torque_x',
  [23] = 'r_hand_torque_y',
  [24] = 'r_hand_torque_z',
  
  -- group ids
  l_foot = {
    'l_foot_force_x',
    'l_foot_force_y',
    'l_foot_force_z',
    'l_foot_torque_x',
    'l_foot_torque_y',
    'l_foot_torque_z',
  },
  r_foot = {
    'r_foot_force_x',
    'r_foot_force_y',
    'r_foot_force_z',
    'r_foot_torque_x',
    'r_foot_torque_y',
    'r_foot_torque_z',
  },
  l_hand = {
    'l_hand_force_x',
    'l_hand_force_y',
    'l_hand_force_z',
    'l_hand_torque_x',
    'l_hand_torque_y',
    'l_hand_torque_z',
  },
  r_hand = {
    'r_hand_force_x',
    'r_hand_force_y',
    'r_hand_force_z',
    'r_hand_torque_x',
    'r_hand_torque_y',
    'r_hand_torque_z',
  },
}

-- ahrs config
---------------------------------------------------------------------------

ahrs = {}

ahrs.id = {
  -- device ids
  [1] = 'x_accel',
  [2] = 'y_accel',
  [3] = 'z_accel',
  [4] = 'x_gyro',
  [5] = 'y_gyro',
  [6] = 'z_gyro',
  [7] = 'x_euler',
  [8] = 'y_euler',
  [9] = 'z_euler',
  
  -- group ids
  accel = { 
    'x_accel',
    'y_accel',
    'z_accel',
  },
  gyro = { 
    'x_gyro',
    'y_gyro',
    'z_gyro',
  },
  euler = { 
    'x_euler',
    'y_euler',
    'z_euler',
  }
}

-- battery config
---------------------------------------------------------------------------

battery = {}

battery.id = {
  -- device ids
  [1] = 'lower_body',
  [2] = 'upper_body',
  [3] = 'computing',
}

-- initialize device indices
----------------------------------------------------------------------------

function create_device_index(device)
  -- create device string index
  device.all = {}
  for i = 1,#device.id do
    device[device.id[i]] = i
    device.all[i] = i
  end
  for k,v in pairs(device.id) do
    if (type(k) == 'string') then
      device[k] = {}
      for i,id in pairs(v) do
         device[k][i] = device[id]
      end
    end
  end
end

for i = 1,#devices do  
  create_device_index(getfenv()[devices[i]])
end
