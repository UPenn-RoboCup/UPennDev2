module(..., package.seeall)

require('controller')
require('Transform')
require('cbuffer')
require('Config')
require('vector')
require('servo')
require('acm')
require('scm')

local controlP = Config.webots.controlP
local maxForce = Config.webots.maxForce
local maxVelocity = Config.webots.maxVelocity
local maxAcceleration = Config.webots.maxAcceleration

local servo_position_bias = Config.bias.servo_position_bias
  or vector.zeros(servo.count)
local servo_force_bias = Config.bias.servo_force_bias
  or vector.zeros(servo.count)

local tags = {} -- webots tags
local timeStep = nil

local servoNames = { -- webots servo names
  'l_hipyaw', 'li_hip_servo', 'lo_hip_servo', 
  'l_knee_servo', 'li_ankle_servo', 'lo_ankle_servo', 
  'r_hipyaw', 'ri_hip_servo', 'ro_hip_servo', 
  'r_knee_servo', 'ri_ankle_servo', 'ro_ankle_servo', 
}

-- Actuator / sensor interface 
----------------------------------------------------------------

local function initialize_devices()
  -- intialize webots devices
  tags.saffir = controller.wb_supervisor_node_get_from_def('SAFFiR')
  tags.gyro = controller.wb_robot_get_device('gyro')
  controller.wb_gyro_enable(tags.gyro, timeStep)
  tags.accel = controller.wb_robot_get_device('accelerometer')
  controller.wb_accelerometer_enable(tags.accel, timeStep)
  tags.l_fts = controller.wb_robot_get_device('left_fts_receiver')
  controller.wb_receiver_enable(tags.l_fts, timeStep)
  tags.r_fts = controller.wb_robot_get_device('right_fts_receiver')
  controller.wb_receiver_enable(tags.r_fts, timeStep)
  tags.servo = {}
  for i = 1,servo.count do
    tags.servo[i] = controller.wb_robot_get_device(servoNames[i])
    controller.wb_servo_enable_position(tags.servo[i], timeStep)
    controller.wb_servo_enable_motor_force_feedback(tags.servo[i], timeStep)
    controller.wb_servo_set_control_p(tags.servo[i], controlP)
    controller.wb_servo_set_motor_force(tags.servo[i], maxForce)
    controller.wb_servo_set_velocity(tags.servo[i], maxVelocity)
    controller.wb_servo_set_acceleration(tags.servo[i], maxAcceleration)
  end
end

local function update_actuators()
  -- update shared memory
  acm:update()
  -- write webots actuator values 
  local mode = acm:get_motor_mode()
  for i = 1,servo.count do
    if (mode[i] == 0) then -- disabled
      controller.wb_servo_set_force(tags.servo[i], 0)
    elseif (mode[i] == 1) then -- position control
      controller.wb_servo_set_position(tags.servo[i], acm:get_motor_position(i))
    elseif (mode[i] == 2) then -- force control
      controller.wb_servo_set_force(tags.servo[i], acm:get_motor_force(i))
    end
  end
end

local function update_sensors()
  -- read webots sensor values
  local mode = acm:get_motor_mode()
  -- update motor sensors
  for i = 1,servo.count do
    if (mode[i] == 0) then
      scm:set_motor_force(0, i)
    elseif (mode[i] == 1) then
      scm:set_motor_force(controller.wb_servo_get_motor_force_feedback(tags.servo[i]), i)
    elseif (mode[i] == 2) then
      scm:set_motor_force(acm:get_motor_force(i), i)
    end
    scm:set_motor_position(controller.wb_servo_get_position(tags.servo[i]), i)
  end
  -- update imu sensors
  local t = {}
  local orientation = controller.wb_supervisor_node_get_orientation(tags.saffir)
  t[1] = vector.new({orientation[1], orientation[2], orientation[3], 0});
  t[2] = vector.new({orientation[4], orientation[5], orientation[6], 0});
  t[3] = vector.new({orientation[7], orientation[8], orientation[9], 0});
  t[4] = vector.new({0, 0, 0, 1});
  local euler_angles = Transform.getEuler(t)
  scm:set_imu_gyro(controller.wb_gyro_get_values(tags.gyro))
  scm:set_imu_accel(controller.wb_accelerometer_get_values(tags.accel))
  scm:set_imu_euler(euler_angles)
  -- update force-torque sensors
  local l_fts, r_fts = {}, {}
  local l_fts_buffer = cbuffer.new('string', controller.wb_receiver_get_data(tags.l_fts)) 
  local r_fts_buffer = cbuffer.new('string', controller.wb_receiver_get_data(tags.r_fts)) 
  for i = 1,6 do
    l_fts[i] = l_fts_buffer:get('double', (i-1)*8)
    r_fts[i] = r_fts_buffer:get('double', (i-1)*8)
  end
  controller.wb_receiver_next_packet(tags.l_fts)
  controller.wb_receiver_next_packet(tags.r_fts)
  scm:set_l_force_torque(l_fts)
  scm:set_r_force_torque(r_fts)
  -- update shared memory
  scm:update()
end

-- User interface
-------------------------------------------------------------------

function entry()
  -- initialize webots devices
  controller.wb_robot_init()
  timeStep = controller.wb_robot_get_basic_time_step()
  initialize_devices()
  -- initialize actuator shared memory 
  acm:set_joint_update_enable(1)
  acm:set_servo_update_enable(1)
  acm:set_joint_write_enable(1, joint.all)
  acm:set_servo_write_enable(1, servo.all)
  acm:set_joint_angle_bias(0, joint.all)
  acm:set_joint_torque_bias(0, joint.all)
  acm:set_servo_position_bias(servo_position_bias)
  acm:set_servo_force_bias(servo_force_bias)
  -- initialize sensor shared memory
  scm:set_joint_update_enable(1)
  scm:set_servo_update_enable(1)
  controller.wb_robot_step(timeStep)
  update_sensors()
  -- initialize actuator commands 
  acm:set_joint_mode(1, servo.all) -- position mode
  acm:set_joint_angle(scm:get_joint_angle())
  acm:set_joint_torque(0, servo.all)
  acm:update()
end

function update()
  update_actuators()
  if (controller.wb_robot_step(timeStep) < 0) then
    os.exit()
  end
  update_sensors()
end

function exit()
end
