require('Transform')
require('cbuffer')
require('Config')
require('webots')
require('vector')
require('util')
require('dcm')

Body = {}

local function limit(x, min, max)
  return math.min(math.max(x, min), max)
end

-- Setup
---------------------------------------------------------------------------

local joint = Config.joint
local simulator_iterations = 2

-- servo controller parameters
local max_force = 100
local max_stiffness = 10000
local max_damping = 0  -- damping disabled due to ODE instability
local max_velocity = 7
local max_acceleration = 70
local velocity_p_gain = 0.1*vector.ones(#joint.id)

local joint_ff_force = vector.zeros(#joint.id)
local joint_p_force = vector.zeros(#joint.id)
local joint_d_force = vector.zeros(#joint.id)

local tags = {} -- webots tags
local time_step = nil

local servoNames = { -- webots servo names
  'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 
  'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll',
  'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 
  'r_knee_pitch', 'r_ankle_pitch', 'r_ankle_roll',
}

-- Actuator / sensor interface 
----------------------------------------------------------------

local function initialize_devices()
  -- intialize webots devices
  tags.robot = webots.wb_supervisor_node_get_from_def('Ash')
  tags.robot_translation = webots.wb_supervisor_node_get_field(
    tags.robot, "translation")
  tags.robot_rotation = webots.wb_supervisor_node_get_field(
    tags.robot, "rotation")
  tags.gyro = webots.wb_robot_get_device('gyro')
  webots.wb_gyro_enable(tags.gyro, time_step)
  tags.accel = webots.wb_robot_get_device('accelerometer')
  webots.wb_accelerometer_enable(tags.accel, time_step)
  tags.physics_receiver = webots.wb_robot_get_device('physics_receiver')
  webots.wb_receiver_enable(tags.physics_receiver, time_step)
  tags.physics_emitter = webots.wb_robot_get_device('physics_emitter')

  tags.servo = {}
  for i = 1,#joint.id do
    tags.servo[i] = webots.wb_robot_get_device(servoNames[i])
    webots.wb_servo_enable_position(tags.servo[i], time_step)
    webots.wb_servo_enable_motor_force_feedback(tags.servo[i], time_step)
    webots.wb_servo_set_control_p(tags.servo[i], 100)
    webots.wb_servo_set_position(tags.servo[i], 1/0)
    webots.wb_servo_set_velocity(tags.servo[i], 0)
    webots.wb_servo_set_motor_force(tags.servo[i], 1e-15)
    webots.wb_servo_set_acceleration(tags.servo[i], max_acceleration)
  end
end

local function update_actuators()
  -- update webots actuator values 
  local joint_enable = dcm:get_joint_enable()
  local joint_force_desired = dcm:get_joint_force()
  local joint_position_desired = dcm:get_joint_position()
  local joint_velocity_desired = dcm:get_joint_velocity()
  local joint_position_actual = dcm:get_joint_position_sensor()
  local joint_velocity_actual = dcm:get_joint_velocity_sensor()
  local joint_stiffness = dcm:get_joint_stiffness()
  local joint_damping = dcm:get_joint_damping()
  local position_error = vector.zeros(#joint.id)
  local velocity_error = vector.zeros(#joint.id)

  -- calculate joint forces
  for i = 1,#joint.id do
    if (joint_enable[i] == 0) then
      -- zero forces
      joint_ff_force[i] = 0 
      joint_d_force[i] = 0
      joint_p_force[i] = 0
      webots.wb_servo_set_force(tags.servo[i], 0)
    else
      -- calculate feedforward force 
      joint_ff_force[i] = joint_force_desired[i]
      joint_ff_force[i] = limit(joint_ff_force[i], -max_force, max_force)
      -- calculate spring force 
      position_error[i] = joint_position_desired[i] - joint_position_actual[i]
      joint_stiffness[i] = limit(joint_stiffness[i], 0, 1)
      joint_p_force[i] = joint_stiffness[i]*max_stiffness*position_error[i]
      joint_p_force[i] = limit(joint_p_force[i], -max_force, max_force)
      -- calculate damping force
      velocity_error[i] = joint_velocity_desired[i] - joint_velocity_actual[i]
      joint_damping[i] = limit(joint_damping[i], 0, 1)
      joint_d_force[i] = joint_damping[i]*max_damping*velocity_error[i]
      joint_d_force[i] = limit(joint_d_force[i], -max_force, max_force)
    end
  end

  -- update spring force using motor velocity controller
  for i = 1,#joint.id do
    local max_servo_force = math.abs(joint_p_force[i])
    local servo_velocity = velocity_p_gain[i]*position_error[i]*(1000/time_step)
    servo_velocity = limit(servo_velocity, -max_velocity, max_velocity)
    webots.wb_servo_set_motor_force(tags.servo[i], max_servo_force)
    webots.wb_servo_set_velocity(tags.servo[i], servo_velocity)
  end

  -- update feedforward and damping forces using physics plugin
  local buffer = cbuffer.new(#joint.id*8)
  for i = 1,#joint.id do
    local servo_force = joint_ff_force[i] + joint_d_force[i]
    servo_force =  limit(servo_force, -max_force, max_force)
    buffer:set('double', servo_force, (i-1)*8)
  end
  webots.wb_emitter_send(tags.physics_emitter, tostring(buffer))
end

local function update_sensors()
  -- update webots sensor values
  local joint_enable = dcm:get_joint_enable()
  for i = 1,#joint.id do
    if (joint_enable[i] == 0) then
      dcm:set_joint_force_sensor(0, i)
    else
      dcm:set_joint_force_sensor(webots.wb_servo_get_motor_force_feedback(
          tags.servo[i]) + joint_ff_force[i], i)
    end
    dcm:set_joint_position_sensor(
        webots.wb_servo_get_position(tags.servo[i]), i)
  end
  
  -- update imu readings
  local t = {}
  local orientation = webots.wb_supervisor_node_get_orientation(tags.robot)
  t[1] = vector.new({orientation[1], orientation[2], orientation[3], 0})
  t[2] = vector.new({orientation[4], orientation[5], orientation[6], 0})
  t[3] = vector.new({orientation[7], orientation[8], orientation[9], 0})
  t[4] = vector.new({0, 0, 0, 1});
  local euler_angles = Transform.getEuler(t)
  dcm:set_ahrs(webots.wb_gyro_get_values(tags.gyro), 'gyro')
  dcm:set_ahrs(webots.wb_accelerometer_get_values(tags.accel), 'accel')
  dcm:set_ahrs(euler_angles, 'euler')

  -- update force-torque readings and joint velocities using physics plugin
  local buffer = cbuffer.new(webots.wb_receiver_get_data(tags.physics_receiver))
  local joint_velocity = {}
  for i = 1,#joint.id do
    joint_velocity[i] = buffer:get('double', (i-1)*8)
  end
  dcm:set_joint_velocity_sensor(joint_velocity)
  local l_fts = {}
  local r_fts = {}
  for i = 1,6 do
    l_fts[i] = buffer:get('double', (i-1)*8 + #joint.id*8)
    r_fts[i] = buffer:get('double', (i-1)*8 + #joint.id*8 + 48)
  end
  dcm:set_force_torque(l_fts, 'l_ankle')
  dcm:set_force_torque(r_fts, 'r_ankle')
  webots.wb_receiver_next_packet(tags.physics_receiver)
end

-- User interface 
---------------------------------------------------------------------------

Body.get_time = webots.wb_robot_get_time

function Body.set_time_step(t)
  -- for compatibility
end

function Body.get_time_step()
  return time_step*simulator_iterations
end

function Body.get_update_rate()
  return 1000/(time_step*simulator_iterations)
end

function Body.set_simulator_pose(pose)
  webots.wb_supervisor_field_set_sf_vec3f(tags.robot_translation,
    {pose[1], pose[2], pose[3]})
  webots.wb_supervisor_field_set_sf_rotation(tags.robot_rotation, 
    {0, 0, 1, pose[4] or 0})
end

function Body.entry()
  -- initialize webots devices
  webots.wb_robot_init()
  time_step = webots.wb_robot_get_basic_time_step()
  initialize_devices()

  -- initialize shared memory 
  dcm:set_joint_enable(1, 'all')
  dcm:set_joint_stiffness(1, 'all') -- position control
  dcm:set_joint_damping(0, 'all')
  dcm:set_joint_force(0, 'all')
  dcm:set_joint_position(0, 'all')
  dcm:set_joint_velocity(0, 'all')
  dcm:set_joint_force_sensor(0, 'all')
  dcm:set_joint_position_sensor(0, 'all')
  dcm:set_joint_velocity_sensor(0, 'all')

  -- initialize sensor shared memory
  Body.update()
end

function Body.update()
  for i = 1, simulator_iterations do
    update_actuators()
    if (webots.wb_robot_step(time_step) < 0) then
      os.exit()
    end
    update_sensors()
  end
end

function Body.exit()
end

return Body
