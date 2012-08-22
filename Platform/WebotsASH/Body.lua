require('Transform')
require('cbuffer')
require('Config')
require('webots')
require('vector')
require('acm')
require('scm')

Body = {}

-- Setup
---------------------------------------------------------------------------

local joint = Config.joint
local controlP = Config.webots.controlP
local maxForce = Config.webots.maxForce
local maxVelocity = Config.webots.maxVelocity
local maxAcceleration = Config.webots.maxAcceleration

local joint_position_bias = Config.bias.joint_position_bias
  or vector.zeros(#joint.id)
local joint_force_bias = Config.bias.joint_force_bias
  or vector.zeros(#joint.id)

local tags = {} -- webots tags
local timeStep = nil

local servoNames = { -- webots servo names
  'l_hipyaw', 'l_hiproll', 'l_hippitch', 
  'l_knee_pivot', 'l_ankle', 'l_ankle_p2',
  'r_hipyaw', 'r_hiproll', 'r_hippitch', 
  'r_knee_pivot', 'r_ankle', 'r_ankle_p2',
}

-- Actuator / sensor interface 
----------------------------------------------------------------

local function initialize_devices()
  -- intialize webots devices
  tags.saffir = webots.wb_supervisor_node_get_from_def('SAFFiR')
  tags.gyro = webots.wb_robot_get_device('gyro')
  webots.wb_gyro_enable(tags.gyro, timeStep)
  tags.accel = webots.wb_robot_get_device('accelerometer')
  webots.wb_accelerometer_enable(tags.accel, timeStep)
  tags.l_fts = webots.wb_robot_get_device('left_fts_receiver')
  webots.wb_receiver_enable(tags.l_fts, timeStep)
  tags.r_fts = webots.wb_robot_get_device('right_fts_receiver')
  webots.wb_receiver_enable(tags.r_fts, timeStep)

  tags.servo = {}
  for i = 1,#joint.id do
    tags.servo[i] = webots.wb_robot_get_device(servoNames[i])
    webots.wb_servo_enable_position(tags.servo[i], timeStep)
    webots.wb_servo_enable_motor_force_feedback(tags.servo[i], timeStep)
    webots.wb_servo_set_control_p(tags.servo[i], controlP)
    webots.wb_servo_set_motor_force(tags.servo[i], maxForce)
    webots.wb_servo_set_velocity(tags.servo[i], maxVelocity)
    webots.wb_servo_set_acceleration(tags.servo[i], maxAcceleration)
  end
end

local function update_actuators()
  -- update webots actuator values 
  local mode = acm:get_joint_mode()
  local enable = acm:get_joint_enable()
  local joint_position = acm:get_joint_position() - joint_position_bias
  local joint_force = acm:get_joint_force() - joint_force_bias

  for i = 1,#joint.id do
    if (enable[i] == 0) then   -- disabled
      webots.wb_servo_set_force(tags.servo[i], 0)
    elseif (mode[i] == 0) then -- position control
      webots.wb_servo_set_position(tags.servo[i], joint_position[i])
    elseif (mode[i] == 1) then -- force control
      webots.wb_servo_set_force(tags.servo[i], joint_force[i])
    end
  end
end

local function update_sensors()
  -- update webots sensor values
  local mode = acm:get_joint_mode()
  local enable = acm:get_joint_enable()
  for i = 1,#joint.id do
    if (enable[i] == 0) then   -- disabled
      scm:set_joint_force(0, i)
    elseif (mode[i] == 0) then -- position control
      scm:set_joint_force(webots.wb_servo_get_motor_force_feedback(tags.servo[i]), i)
    elseif (mode[i] == 1) then -- force control
      scm:set_joint_force(acm:get_joint_force(i), i)
    end
    scm:set_joint_position(webots.wb_servo_get_position(tags.servo[i]), i)
  end

  -- update imu readings
  local t = {}
  local orientation = webots.wb_supervisor_node_get_orientation(tags.saffir)
  t[1] = vector.new({orientation[1], orientation[2], orientation[3], 0})
  t[2] = vector.new({orientation[4], orientation[5], orientation[6], 0})
  t[3] = vector.new({orientation[7], orientation[8], orientation[9], 0})
  t[4] = vector.new({0, 0, 0, 1});
  local euler_angles = Transform.getEuler(t)
  scm:set_ahrs(webots.wb_gyro_get_values(tags.gyro), 'accel')
  scm:set_ahrs(webots.wb_accelerometer_get_values(tags.accel), 'gyro')
  scm:set_ahrs(euler_angles, 'euler')

  -- update force-torque readings
  local l_fts, r_fts = {}, {}
  local l_fts_buffer = cbuffer.new('string', webots.wb_receiver_get_data(tags.l_fts)) 
  local r_fts_buffer = cbuffer.new('string', webots.wb_receiver_get_data(tags.r_fts)) 
  for i = 1,6 do
    l_fts[i] = l_fts_buffer:get('double', (i-1)*8)
    r_fts[i] = r_fts_buffer:get('double', (i-1)*8)
  end
  webots.wb_receiver_next_packet(tags.l_fts)
  webots.wb_receiver_next_packet(tags.r_fts)
  scm:set_force_torque(l_fts, 'l_ankle')
  scm:set_force_torque(r_fts, 'r_ankle')
end


-- User interface 
---------------------------------------------------------------------------

Body.get_time = webots.wb_robot_get_time

function Body.entry()
  -- initialize webots devices
  webots.wb_robot_init()
  timeStep = webots.wb_robot_get_basic_time_step()
  initialize_devices()

  -- initialize sensor shared memory
  webots.wb_robot_step(timeStep)
  update_sensors()

  -- initialize actuator commands 
  acm:set_joint_mode(0, 'all') -- position mode
  acm:set_joint_enable(1, 'all')
  acm:set_joint_position(scm:get_joint_position())
  acm:set_joint_force(0, 'all')
  Body.update()
end

function Body.update()
  update_actuators()
  if (webots.wb_robot_step(timeStep) < 0) then
    os.exit()
  end
  update_sensors()
end

function Body.exit()
end

return Body
