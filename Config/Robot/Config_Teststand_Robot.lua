module(..., package.seeall)

-- joint config
---------------------------------------------------------------------------

joint = {}

joint.id = { 
  [1] = 'l_knee_pitch',
}

joint.index = {}
for i,k in pairs(joint.id) do
  joint.index[k] = i
end

joint.index.all = {}
for i = 1,#joint.id do
  joint.index.all[i] = i
end

joint.index.l_hip = {
}

joint.index.r_hip = {
}

joint.index.hips = {
}

joint.index.l_knee = {
  joint.index.l_knee_pitch,
}

joint.index.r_knee = {
}

joint.index.knees = {
  joint.index.l_knee_pitch,
}

joint.index.l_ankle = {
}

joint.index.r_ankle = {
}

joint.index.ankles = {
}

joint.index.l_leg = {
  joint.index.l_knee_pitch,
}

joint.index.r_leg = {
}

joint.index.legs = {
  joint.index.l_knee_pitch,
}

joint.index.torso = {
}

joint.index.l_arm = {
}

joint.index.r_arm = {
}

joint.index.arms = {
}

joint.index.head = {
}

-- motor config
---------------------------------------------------------------------------

motor = {}

motor.id = { 
  [1] = 'l_knee_pitch',
}

motor.index = {}
for i,k in pairs(motor.id) do
  motor.index[k] = i
end

motor.index.all = {}
for i = 1,#motor.id do
  motor.index.all[i] = i
end

motor.index.l_hip = {
}

motor.index.r_hip = {
}

motor.index.hips = {
}

motor.index.l_knee = {
  motor.index.l_knee_pitch,
}

motor.index.r_knee = {
}

motor.index.knees = {
  motor.index.l_knee_pitch,
}

motor.index.l_ankle = {
}

motor.index.r_ankle = {
}

motor.index.ankles = {
}

motor.index.l_leg = {
  motor.index.l_knee_pitch,
}

motor.index.r_leg = {
}

motor.index.legs = {
  motor.index.l_knee_pitch,
}

motor.index.torso = {
}

motor.index.l_arm = {
}

motor.index.r_arm = {
}

motor.index.arms = {
}

motor.index.head = {
}

-- force_torque config
---------------------------------------------------------------------------

force_torque = {}

force_torque.id = {
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
}

force_torque.index = {}
for i,k in pairs(force_torque.id) do
  force_torque.index[k] = i
end

force_torque.index.all = {}
for i = 1,#force_torque.id do
  force_torque.index.all[i] = i
end

force_torque.index.l_ankle_force = {
  force_torque.index.l_ankle_force_x,
  force_torque.index.l_ankle_force_y,
  force_torque.index.l_ankle_force_z,
}

force_torque.index.l_ankle_torque = {
  force_torque.index.l_ankle_torque_x,
  force_torque.index.l_ankle_torque_y,
  force_torque.index.l_ankle_torque_z,
}

force_torque.index.l_ankle = {
  force_torque.index.l_ankle_force_x,
  force_torque.index.l_ankle_force_y,
  force_torque.index.l_ankle_force_z,
  force_torque.index.l_ankle_torque_x,
  force_torque.index.l_ankle_torque_y,
  force_torque.index.l_ankle_torque_z,
}

force_torque.index.r_ankle_force = {
  force_torque.index.r_ankle_force_x,
  force_torque.index.r_ankle_force_y,
  force_torque.index.r_ankle_force_z,
}

force_torque.index.r_ankle_torque = {
  force_torque.index.r_ankle_torque_x,
  force_torque.index.r_ankle_torque_y,
  force_torque.index.r_ankle_torque_z,
}

force_torque.index.r_ankle = {
  force_torque.index.r_ankle_force_x,
  force_torque.index.r_ankle_force_y,
  force_torque.index.r_ankle_force_z,
  force_torque.index.r_ankle_torque_x,
  force_torque.index.r_ankle_torque_y,
  force_torque.index.r_ankle_torque_z,
}

-- tactile_array config
---------------------------------------------------------------------------
tactile_array = {}

tactile_array.id = {
  [1] = 'l_thumb',
  [2] = 'l_index_finger',
  [3] = 'l_middle_finger',
  [4] = 'r_thumb',
  [5] = 'r_index_finger',
  [6] = 'r_middle_finger',
}

tactile_array.index = {}
for i,k in pairs(tactile_array.id) do
  tactile_array.index[k] = i
end

tactile_array.index.all = {}
for i = 1,#tactile_array.id do
  tactile_array.index.all[i] = i
end

tactile_array.index.l_hand = {
  tactile_array.index.l_thumb,
  tactile_array.index.l_index_finger,
  tactile_array.index.l_middle_finger,
}

tactile_array.index.r_hand = {
  tactile_array.index.r_thumb,
  tactile_array.index.r_index_finger,
  tactile_array.index.r_middle_finger,
}


-- ahrs config
---------------------------------------------------------------------------

ahrs = {}

ahrs.id = {
  [1] = 'x_accel',
  [2] = 'y_accel',
  [3] = 'z_accel',
  [4] = 'x_gyro',
  [5] = 'y_gyro',
  [6] = 'z_gyro',
  [7] = 'x_euler',
  [8] = 'y_euler',
  [9] = 'z_euler',
}

ahrs.index = {}
for i,k in pairs(ahrs.id) do
  ahrs.index[k] = i
end

ahrs.index.all = {}
for i = 1,#ahrs.id do
  ahrs.index.all[i] = i
end

ahrs.index.accel = { 
  ahrs.index.x_accel,
  ahrs.index.y_accel,
  ahrs.index.z_accel,
}

ahrs.index.gyro = { 
  ahrs.index.x_gyro,
  ahrs.index.y_gyro,
  ahrs.index.z_gyro,
}

ahrs.index.euler = { 
  ahrs.index.x_euler,
  ahrs.index.y_euler,
  ahrs.index.z_euler,
}

-- battery config
---------------------------------------------------------------------------

battery = {}

battery.id = {
  [1] = 'lower_body',
  [2] = 'upper_body',
  [3] = 'computing',
}

battery.index = {}
for i,k in pairs(battery.id) do
  battery.index[k] = i
end

battery.index.all = {}
for i = 1,#battery.id do
  battery.index.all[i] = i
end
