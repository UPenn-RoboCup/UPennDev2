require('dcm')
require('zmq')

vrep_child_script = {}

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

local zmq_context
local frame_update_socket

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
  handles.robot = simGetObjectHandle('torso')
  handles.gyro = simTubeOpen(0, 'gyroData'..simGetNameSuffix(nil), 10)
  handles.accel = simTubeOpen(0, 'accelerometerData'..simGetNameSuffix(nil), 10)

  handles.servo = {}
  for i = 1,N_JOINT do
    handles.servo[i] = simGetObjectHandle(servoNames[i])
    if handles.servo[i] < 0 then
      print('Could not get handle for '..servoNames[i])
    end
    -- Make joint motion cyclic (unbounded). Third argument is ignored.
    simSetJointInterval(handles.servo[i], true, {0, 0})
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
    
    -- Object attribute 2000 is whether the motor is enabled
    simSetObjectIntParameter(handles.servo[i], 2000, joint_enable[i])
    
    -- Object attribute 2001 is whether the motor is position-controlled
    if joint_stiffness[i] then
      simSetJointForce(handles.servo[i], 10000)
      simSetObjectIntParameter(handles.servo[i], 2001, joint_stiffness[i])
      simSetJointTargetPosition(handles.servo[i], joint_position_desired[i])
      -- Object attributes 2002-2004 are PID values
      simSetObjectFloatParameter(handles.servo[i], 2002, 20000)
    end
  end
end

local function update_sensors()
  -- update VRep sensor values
  local joint_enable = dcm:get_joint_enable()
  for i = 1,N_JOINT do
    f = simJointGetForce(handles.servo[i])
    dcm:set_joint_force_sensor(f, i)
    
    ret_val, velocity = simGetObjectFloatParameter(handles.servo[i], 2012)
    dcm:set_joint_velocity_sensor(velocity, i)
    
    dcm:set_joint_position_sensor(
        simGetJointPosition(handles.servo[i]), i)
  end

  -- update imu readings
  local euler_angles = simGetObjectOrientation(handles.robot, -1)
  dcm:set_ahrs(euler_angles, 'euler')
  
  local data = simTubeRead(handles.gyro)
  if (data) then
    floats = simUnpackFloats(data)
    dcm:set_ahrs(floats, 'gyro')
  end
  
  data = simTubeRead(handles.accel)
  if (data) then
    floats = simUnpackFloats(data)
    dcm:set_ahrs(floats, 'accel')
  end

  -- update force-torque readings
  -- dcm:set_force_torque(l_fts, 'l_foot')
  -- dcm:set_force_torque(r_fts, 'r_foot')
end

-- User interface
---------------------------------------------------------------------------

vrep_child_script.get_time = simGetSimulationTime

function vrep_child_script.set_time_step(t)
  -- for compatibility
end

function vrep_child_script.get_time_step()
  return time_step_ms/1000
end

function vrep_child_script.get_update_rate()
  return 1000/time_step_ms
end

function vrep_child_script.reset_simulator()
  simStopSimulation()
  simStartSimulation()
end

function vrep_child_script.reset_simulator_physics()
  simResetDynamicObject(handles.robot)
end

function vrep_child_script.set_simulator_torso_frame(frame)
  local pose = frame:get_pose()
  -- -1 means set absolute position/orientation
  simSetObjectPosition(handles.robot, -1, {pose[1], pose[2], pose[3]})
  simSetObjectOrientation(handles.robot, -1, {pose[4], pose[5], pose[6]})
end

function vrep_child_script.set_simulator_torso_twist(twist)
end

function vrep_child_script.entry()
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

  -- initialize motion manager
  zmq_context = zmq.init(1)
  frame_update_socket = zmq_context:socket(zmq.PUB)
  frame_update_socket:bind("tcp://127.0.0.1:12000")

  -- initialize sensor shared memory
  vrep_child_script.update()
end

function vrep_child_script.update()
  update_actuators()
  update_sensors()
  local msg = "f"..simGetSimulationTime()
  frame_update_socket:send(msg)
end

function vrep_child_script.exit()
  frame_update_socket:close()
  zmq_context:term()
end

return vrep_child_script
