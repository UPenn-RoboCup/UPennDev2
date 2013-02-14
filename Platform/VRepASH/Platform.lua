require('Transform')
require('cbuffer')
require('Config')
require('vector')
require('util')
require('dcm')

Platform = {}

local function update_sensors()
  -- update webots sensor values
  local joint_enable = dcm:get_joint_enable()
  for i = 1,N_JOINT do
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
  local euler_angles = Transform.get_euler(t)
  dcm:set_ahrs(webots.wb_gyro_get_values(tags.gyro), 'gyro')
  dcm:set_ahrs(webots.wb_accelerometer_get_values(tags.accel), 'accel')
  dcm:set_ahrs(euler_angles, 'euler')

  -- update force-torque readings and joint velocities using physics plugin
  local buffer = cbuffer.new(webots.wb_receiver_get_data(tags.physics_receiver))
  local joint_velocity = {}
  for i = 1,N_JOINT do
    joint_velocity[i] = buffer:get('double', (i-1)*8)
  end
  dcm:set_joint_velocity_sensor(joint_velocity)
  local l_fts = {}
  local r_fts = {}
  for i = 1,6 do
    l_fts[i] = buffer:get('double', (i-1)*8 + N_JOINT*8)
    r_fts[i] = buffer:get('double', (i-1)*8 + N_JOINT*8 + 48)
  end
  dcm:set_force_torque(l_fts, 'l_foot')
  dcm:set_force_torque(r_fts, 'r_foot')
  webots.wb_receiver_next_packet(tags.physics_receiver)
end

-- User interface
---------------------------------------------------------------------------

Platform.get_time = simGetSimulationTime

function Platform.set_time_step(t)
  -- for compatibility
end

function Platform.get_time_step()
  return time_step*simulator_iterations/1000
end

function Platform.get_update_rate()
  return 1000/(time_step*simulator_iterations)
end

function Platform.reset_simulator()
  webots.wb_supervisor_simulation_revert()
end

function Platform.reset_simulator_physics()
  webots.wb_supervisor_simulation_physics_reset()
end

function Platform.set_simulator_torso_frame(frame)
  local pose = frame:get_pose6D()
  webots.wb_supervisor_field_set_sf_vec3f(tags.robot_translation,
    {pose[1], pose[2], pose[3]})
  webots.wb_supervisor_field_set_sf_rotation(tags.robot_rotation,
    {0, 0, 1, pose[4]})
end

function Platform.set_simulator_torso_twist(twist)
  torso_twist = twist
  torso_twist_updated = 1
end

function Platform.entry()
  -- initialize webots devices
  webots.wb_robot_init()
  time_step = simGetSimulationTimeStep()
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
  Platform.update()
end

function Platform.update()
  for i = 1, simulator_iterations do
    update_actuators()
    if (webots.wb_robot_step(time_step) < 0) then
      os.exit()
    end
    update_sensors()
  end
end

function Platform.exit()
end

return Platform
