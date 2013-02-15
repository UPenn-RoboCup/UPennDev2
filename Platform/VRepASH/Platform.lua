require('Transform')
require('cbuffer')
require('Config')
require('vector')
require('util')
require('dcm')

Platform = {}

local function limit(x, min, max)
  return math.min(math.max(x, min), max)
end

local function sign(x)
  if x > 0 then
    return 1
  elseif x < 0 then
    return -1
  else
    return 0
  end
end

-- Setup
---------------------------------------------------------------------------

local N_JOINT = 12
local handles = {} -- vrep handles
local time_step_ms = nil

local servoNames = { -- vrep servo names
  'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch',
  'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll',
  'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch',
  'r_knee_pitch', 'r_ankle_pitch', 'r_ankle_roll',
}

-- Actuator / sensor interface
----------------------------------------------------------------

local function initialize_devices()
  -- intialize vrep devices
  handles.robot = simGetObjectHandle('Ash')
  handles.gyro = simTubeOpen(0, 'gyroData#')
  handles.accel = simTubeOpen(0, 'accelerometerData#')

  handles.servo = {}
  for i = 1,N_JOINT do
    handles.servo[i] = simGetObjectHandle(servoNames[i])
    -- make joint motion cyclic (unbounded)
    simSetJointInterval(handles.servo[i], true, nil)
    simSetJointTargetVelocity(handles.servo[i], 0)
    simSetJointForce(handles.servo[i], 0)
  end
end

local function update_actuators()
  -- update vrep actuator values
  local joint_enable = dcm:get_joint_enable()
  local joint_force_desired = dcm:get_joint_force()
  local joint_position_desired = dcm:get_joint_position()
  local joint_velocity_desired = dcm:get_joint_velocity()
  local joint_position_actual = dcm:get_joint_position_sensor()
  local joint_velocity_actual = dcm:get_joint_velocity_sensor()
  local joint_stiffness = dcm:get_joint_stiffness()
  local joint_damping = dcm:get_joint_damping()

  -- update spring force using motor velocity controller
  for i = 1,N_JOINT do
    local max_servo_force = math.abs(joint_force_desired[i])
    simSetJointForce(handles.servo[i], max_servo_force)
    simSetJointTargetVelocity(handles.servo[i], joint_velocity_desired[i])
  end
end

local function update_sensors()
  -- update VRep sensor values
  local joint_enable = dcm:get_joint_enable()
  for i = 1,N_JOINT do
    if (joint_enable[i] == 0) then
      dcm:set_joint_force_sensor(0, i)
    else
      dcm:set_joint_force_sensor(
        simJointGetForce(handles.servo[i]), i)
    end
    
    dcm:set_joint_position_sensor(
        simGetJointPosition(handles.servo[i]), i)
  end

  -- update imu readings
  local euler_angles = simGetObjectOrientation(handles.robot)
  dcm:set_ahrs(euler_angles, 'euler')
  
  local data = simTubeRead(handles.gyro)
  if (data) then
    dcm:set_ahrs(simUnpackFloats(data), 'gyro')
  end
  
  data = simTubeRead(handles.accel)
  if (data) then
    dcm:set_ahrs(simUnpackFloats(data), 'accel')
  end

  -- update force-torque readings
  -- dcm:set_force_torque(l_fts, 'l_foot')
  -- dcm:set_force_torque(r_fts, 'r_foot')
end

-- User interface
---------------------------------------------------------------------------

Platform.get_time = simGetSimulationTime

function Platform.set_time_step(t)
  -- for compatibility
end

function Platform.get_time_step()
  return time_step_ms/1000
end

function Platform.get_update_rate()
  return 1000/time_step_ms
end

function Platform.reset_simulator()
  simStopSimulation()
  simStartSimulation()
end

function Platform.reset_simulator_physics()
  simResetDynamicObject(handles.robot)
end

function Platform.set_simulator_torso_frame(frame)
  local pose = frame:get_pose6D()
  -- -1 means set absolute position/orientation
  simSetObjectPosition(handles.robot, -1, {pose[1], pose[2], pose[3]})
  simSetObjectOrientation(handles.robot, -1, {pose[4], pose[5], pose[6]})
end

function Platform.set_simulator_torso_twist(twist)
end

function Platform.entry()
  -- initialize vrep devices
  initialize_devices()
  time_step_ms = simGetSimulationTimeStep()

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
  update_actuators()
  update_sensors()
end

function Platform.exit()
end

return Platform
