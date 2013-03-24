require('pid')

local MAX_FORCE = 300
local MAX_VELOCITY = 10
local MAX_ACCEL = 600
local P_GAIN_FACTOR = 500
local I_GAIN_FACTOR = 500
local D_GAIN_FACTOR = 0
local D_CORNER_FREQUENCY = 30

vrep_position_controller = {}
vrep_position_controller.__index = vrep_position_controller

function vrep_position_controller.new(...)
  -- Following data is handed over from V-REP:
  ------------------------------------------------------------------------------
  local init, revolute, cyclic, jointHandle, passCnt, totalPasses,
  currentPos, targetPos, errorValue, effort, dynStepSize, lowLimit,
  hightLimit, targetVel, targetForce, velUpperLimit = ...

  local o = {}
  o.joint_id = simUnpackFloats(simGetObjectCustomData(jointHandle, 2030))[1]
  o.position_pid = pid.new(dynStepSize)
  return setmetatable(o, vrep_position_controller)
end

function vrep_position_controller:update(...)
  -- Following data is handed over from V-REP:
  ------------------------------------------------------------------------------
  local init, revolute, cyclic, jointHandle, passCnt, totalPasses,
  currentPos, targetPos, errorValue, effort, dynStepSize, lowLimit,
  hightLimit, targetVel, targetForce, velUpperLimit = ...

  -- Initialize position pid controller:
  ------------------------------------------------------------------------------
  if (init) then
    self.position_pid:reset()
    self.position_pid:set_time_step(dynStepSize)
    self.position_pid:set_output_limits(-MAX_VELOCITY, MAX_VELOCITY)
    self.position_pid:set_d_corner_frequency(POSITION_D_CORNER_FREQUENCY)
  end

  -- Update pid gains and setpoints:
  ------------------------------------------------------------------------------
  if (passCnt == 1) then
    local joint_param = simGetObjectCustomData(jointHandle, 2050)
    joint_param = simUnpackFloats(joint_param)
    self.force_setpoint = joint_param[1]
    self.position_setpoint = joint_param[2]
    self.velocity_setpoint = joint_param[3]
    self.p_gain = P_GAIN_FACTOR*joint_param[4]
    self.i_gain = I_GAIN_FACTOR*joint_param[5]
    self.d_gain = D_GAIN_FACTOR*joint_param[6]
    self.position_pid:set_gains(
      self.p_gain,
      self.i_gain,
      0
    )
    self.position_pid:set_setpoint(self.position_setpoint)
  end
 
  -- Get velocity setpoint (limit velocity to prevent overshoot):
  ------------------------------------------------------------------------------
  local velocity_command = self.position_pid:update(currentPos)
  local velocity_current = self.position_pid:get_derivative() 
  local accel_command = velocity_command - velocity_current
  if (accel_command > 0) then
    velocity_command = velocity_current + math.min(accel_command, MAX_ACCEL)
  else
    velocity_command = velocity_current + math.max(accel_command,-MAX_ACCEL)
  end
  velocity_command = math.max(velocity_command,-MAX_VELOCITY)
  velocity_command = math.min(velocity_command, MAX_VELOCITY)
  return MAX_FORCE, velocity_command
end

return vrep_position_controller
