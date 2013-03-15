require('dcm')
require('zmq')
require('cmsgpack')

vrep_child_script = {}

local context = nil
local time_socket = nil
local time_endpoint = 'tcp://127.0.0.1:12000'

local handles = {} -- vrep handles
local servoNames = { -- vrep servo names
  'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch',
  'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll',
  'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch',
  'r_knee_pitch', 'r_ankle_pitch', 'r_ankle_roll',
}

local function initialize_devices()
  -- intialize vrep devices
  handles.robot = simGetObjectHandle('torso')
  handles.gyro = simTubeOpen(0, 'gyroData'..simGetNameSuffix(nil), 10)
  handles.accel = simTubeOpen(0, 'accelerometerData'..simGetNameSuffix(nil), 10)
  handles.servo = {}
  for i = 1,#servoNames do
    handles.servo[i] = simGetObjectHandle(servoNames[i])
    if handles.servo[i] < 0 then
      print('Could not get handle for '..servoNames[i])
    end
    simSetJointInterval(handles.servo[i], true, {0, 0})
  end
end

local function update_actuators()
  -- update joint parameters 
  local joint_enable = dcm:get_joint_enable()
  local joint_force = dcm:get_joint_force()
  local joint_position = dcm:get_joint_position()
  local joint_velocity = dcm:get_joint_velocity()
  local joint_position_p_gain = dcm:get_joint_position_p_gain()
  local joint_position_i_gain = dcm:get_joint_position_i_gain()
  local joint_position_d_gain = dcm:get_joint_position_d_gain()
  local joint_velocity_p_gain = dcm:get_joint_velocity_p_gain()

  for i = 1,#servoNames do
    -- TODO : add joint_velocity_p_gain
    simSetObjectIntParameter(handles.servo[i], 2000, joint_enable[i])
    simSetObjectFloatParameter(handles.servo[i], 2002, joint_position_p_gain[i])
    simSetObjectFloatParameter(handles.servo[i], 2003, joint_position_i_gain[i]) 
    simSetObjectFloatParameter(handles.servo[i], 2004, joint_position_d_gain[i])
    simSetJointForce(handles.servo[i], joint_force[i])
    simSetJointTargetPosition(handles.servo[i], joint_position[i])
    simSetJointTargetVelocity(handles.servo[i], joint_velocity[i])
  end
end

local function update_sensors()
  -- update joint sensors
  for i = 1,#servoNames do
    local force = simJointGetForce(handles.servo[i])
    local position = simGetJointPosition(handles.servo[i])
    local _, velocity = simGetObjectFloatParameter(handles.servo[i], 2012)
    dcm:set_joint_force_sensor(force, i)
    dcm:set_joint_position_sensor(position, i)
    dcm:set_joint_velocity_sensor(velocity, i)
  end

  -- update ahrs sensors
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

local function reset_simulator()
  simStopSimulation()
  simStartSimulation()
end

local function reset_simulator_physics()
  simResetDynamicObject(handles.robot)
end

local function set_simulator_torso_frame(frame)
  local pose = frame:get_pose()
  -- -1 means set absolute position/orientation
  simSetObjectPosition(handles.robot, -1, {pose[1], pose[2], pose[3]})
  simSetObjectOrientation(handles.robot, -1, {pose[4], pose[5], pose[6]})
end

local function set_simulator_torso_twist(twist)
end

function vrep_child_script.entry()
  -- initialize vrep devices
  initialize_devices()

  -- initialize shared memory
  dcm:set_joint_enable(1, 'all')
  dcm:set_joint_position_p_gain(1, 'all') -- position control
  dcm:set_joint_position_i_gain(0, 'all') -- position control
  dcm:set_joint_position_d_gain(0, 'all')
  dcm:set_joint_velocity_p_gain(0, 'all')
  dcm:set_joint_force(0, 'all')
  dcm:set_joint_position(0, 'all')
  dcm:set_joint_velocity(0, 'all')
  dcm:set_joint_force_sensor(0, 'all')
  dcm:set_joint_position_sensor(0, 'all')
  dcm:set_joint_velocity_sensor(0, 'all')

  -- initialize ipc
  context = zmq.init(1)
  time_socket = context:socket(zmq.PUB)
  time_socket:bind(time_endpoint)

  -- initialize sensor shared memory
  vrep_child_script.update()
end

function vrep_child_script.update()
  update_actuators()
  update_sensors()
  local msg = cmsgpack.pack{simGetSimulationTime(), simGetSimulationTimeStep()}
  time_socket:send('time'..msg)
end

function vrep_child_script.exit()
  time_socket:close()
  context:term()
end

return vrep_child_script
