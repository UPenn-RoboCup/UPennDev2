module(..., package.seeall)

devices = {
  'joint',
  'motor',
  'force_torque',
  'tactile_array',
  'ahrs',
  'battery',
}

-- joint config
---------------------------------------------------------------------------

joint = {}

joint.id = { 
  -- device ids
  [1] = 'r_shoulder_pitch',
  [2] = 'r_shoulder_roll',
  [3] = 'r_shoulder_yaw',
  [4] = 'r_elbow_pitch',
  [5] = 'r_wrist_yaw',
  [6] = 'r_wrist_roll',
  [7] = 'r_gripper_thumb',
  [8] = 'r_gripper_middle',
  [9] = 'r_gripper_index',

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
  },
  r_arm = {
  },
  arms = {
  },
  head = {
  },
}

-- motor config
---------------------------------------------------------------------------

motor = {}

motor.id = { 
  -- device ids
  [1] = 'l_knee_pitch',

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
  },
  r_arm = {
  },
  arms = {
  },
  head = {
  },
}

-- force_torque config
---------------------------------------------------------------------------

force_torque = {}

force_torque.id = {
  -- device ids
  [1] = 'l_ankle_force_x',
  [2] = 'l_ankle_force_y',
  [3] = 'l_ankle_force_z',
  [4] = 'l_ankle_torque_x',
  [5] = 'l_ankle_torque_y',
  [6] = 'r_ankle_torque_z',
  [7] = 'r_ankle_force_x',
  [8] = 'r_ankle_force_y',
  [9] = 'r_ankle_force_z',
  [10] = 'r_ankle_torque_x',
  [11] = 'r_ankle_torque_y',
  [12] = 'r_ankle_torque_z',
  
  -- group ids
  l_ankle_force = {
    'l_ankle_force_x',
    'l_ankle_force_y',
    'l_ankle_force_z',
  },
  l_ankle_torque = {
    'l_ankle_torque_x',
    'l_ankle_torque_y',
    'l_ankle_torque_z',
  },
  l_ankle = {
    'l_ankle_force_x',
    'l_ankle_force_y',
    'l_ankle_force_z',
    'l_ankle_torque_x',
    'l_ankle_torque_y',
    'l_ankle_torque_z',
  },
  r_ankle_force = {
    'r_ankle_force_x',
    'r_ankle_force_y',
    'r_ankle_force_z',
  },
  r_ankle_torque = {
    'r_ankle_torque_x',
    'r_ankle_torque_y',
    'r_ankle_torque_z',
  },
  r_ankle = {
    'r_ankle_force_x',
    'r_ankle_force_y',
    'r_ankle_force_z',
    'r_ankle_torque_x',
    'r_ankle_torque_y',
    'r_ankle_torque_z',
  },
}


-- tactile_array config
---------------------------------------------------------------------------
tactile_array = {}

tactile_array.id = {
  -- device ids
  [1] = 'l_thumb',
  [2] = 'l_index_finger',
  [3] = 'l_middle_finger',
  [4] = 'r_thumb',
  [5] = 'r_index_finger',
  [6] = 'r_middle_finger',

  -- group ids
  l_hand = {
    'l_thumb',
    'l_index_finger',
    'l_middle_finger',
  },
  r_hand = {
    'r_thumb',
    'r_index_finger',
    'r_middle_finger',
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
