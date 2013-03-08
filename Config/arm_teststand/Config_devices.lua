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
  [1] = 'l_shoulder_pitch',
  [2] = 'l_shoulder_roll',
  [3] = 'l_shoulder_yaw',
  [4] = 'l_elbow_pitch',
  [5] = 'l_wrist_yaw',
  [6] = 'l_wrist_roll',
  [7] = 'r_shoulder_pitch',
  [8] = 'r_shoulder_roll',
  [9] = 'r_shoulder_yaw',
  [10] = 'r_elbow_pitch',
  [11] = 'r_wrist_yaw',
  [12] = 'r_wrist_roll',
  [13] = 'l_thumb',
  [14] = 'l_finger1',
  [15] = 'l_finger2',
  [16] = 'r_thumb',
  [17] = 'r_finger1',
  [18] = 'r_finger2',

 -- group ids
  l_hip = {
  },
  r_hip = {
  },
  hips = {
  },
  l_knee = {
  },
  r_knee = {
  },
  knees = {
  },
  l_ankle = {
  },
  r_ankle = {
  },
  ankles = {
  },
  l_leg = {
  },
  r_leg = {
  },
  legs = {
  },
  torso = {
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
    'l_thumb',
    'l_finger1',
    'l_finger2',
  },
  r_hand = {
    'r_thumb',
    'r_finger1',
    'r_finger2',
  },
  hands = {
    'l_thumb',
    'l_finger1',
    'l_finger2',
    'r_thumb',
    'r_finger1',
    'r_finger2',
  },
  head = {
  },
  upperbody = {
  },
}

-- motor config
---------------------------------------------------------------------------

motor = {}

motor.id = { 
  -- device ids
  [1] = 'l_shoulder_pitch',
  [2] = 'l_shoulder_roll',
  [3] = 'l_shoulder_yaw',
  [4] = 'l_elbow_pitch',
  [5] = 'l_wrist_yaw',
  [6] = 'l_wrist_roll',
  [7] = 'r_shoulder_pitch',
  [8] = 'r_shoulder_roll',
  [9] = 'r_shoulder_yaw',
  [10] = 'r_elbow_pitch',
  [11] = 'r_wrist_yaw',
  [12] = 'r_wrist_roll',
  [13] = 'l_thumb',
  [14] = 'l_finger1',
  [15] = 'l_finger2',
  [16] = 'r_thumb',
  [17] = 'r_finger1',
  [18] = 'r_finger2',
  
  -- group ids
  l_hip = {
  },
  r_hip = {
  },
  hips = {
  },
  l_knee = {
  },
  r_knee = {
  },
  knees = {
  },
  l_ankle = {
  },
  r_ankle = {
  },
  ankles = {
  },
  l_leg = {
  },
  r_leg = {
  },
  legs = {
  },
  torso = {
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
    'l_thumb',
    'l_finger1',
    'l_finger2',
  },
  r_hand = {
    'r_thumb',
    'r_finger1',
    'r_finger2',
  },
  hands = {
    'l_thumb',
    'l_finger1',
    'l_finger2',
    'r_thumb',
    'r_finger1',
    'r_finger2',
  },
  head = {
  },
  upperbody = {
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
  device.index = {}
  device.index.all = {}
  -- create device indices
  for i = 1,#device.id do
    device.index[device.id[i]] = i 
    device.index.all[i] = i
  end
  -- create group indices
  for k,v in pairs(device.id) do
    if (type(k) == 'string') then
      device.index[k] = {}
      for i,id in pairs(v) do
         device.index[k][i] = device.index[id]
      end
    end
  end
end

for i = 1,#devices do  
  create_device_index(getfenv()[devices[i]])
end
