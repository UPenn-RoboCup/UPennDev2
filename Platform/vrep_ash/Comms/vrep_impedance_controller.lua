require('pid')

local MAX_FORCE = 300
local MAX_VELOCITY = 10
local MAX_ACCEL = 600
local POSITION_P_GAIN_FACTOR = 10000
local POSITION_I_GAIN_FACTOR = 10000
local POSITION_D_GAIN_FACTOR = 10000
local POSITION_D_CORNER_FREQUENCY = 30
local VELOCITY_P_GAIN_FACTOR = 10000

vrep_impedance_controller = {}
vrep_impedance_controller.__index = vrep_impedance_controller

function vrep_impedance_controller.new(...)
  -- Following data is handed over from V-REP:
  ------------------------------------------------------------------------------
  local init, revolute, cyclic, jointHandle, passCnt, totalPasses,
  currentPos, targetPos, errorValue, effort, dynStepSize, lowLimit,
  hightLimit, targetVel, targetForce, velUpperLimit = ...

  local o = {}
  o.joint_id = simUnpackFloats(simGetObjectCustomData(jointHandle, 2030))[1]
  o.position_pid = pid.new(dynStepSize)
  return setmetatable(o, vrep_impedance_controller)
end

function vrep_impedance_controller:update(...)
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
    self.position_pid:set_output_limits(-MAX_FORCE, MAX_FORCE)
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
    self.position_p_gain = POSITION_P_GAIN_FACTOR*joint_param[4]
    self.position_i_gain = POSITION_I_GAIN_FACTOR*joint_param[5]
    self.position_d_gain = POSITION_D_GAIN_FACTOR*joint_param[6]
    self.velocity_p_gain = VELOCITY_P_GAIN_FACTOR*joint_param[7]
    self.position_pid:set_gains(
      self.position_p_gain,
      self.position_i_gain,
      self.position_d_gain
    )
    self.position_pid:set_setpoint(self.position_setpoint)
  end

  -- Get force setpoint:
  ------------------------------------------------------------------------------
  local force_command = self.position_pid:update(currentPos)
  local velocity_current = self.position_pid:get_derivative() 
  local velocity_command = 0
  force_command = force_command
                + self.velocity_p_gain*(self.velocity_setpoint - velocity_current)
                + self.force_setpoint
  if (force_command > 0) then
    velocity_command = velocity_current + MAX_ACCEL*dynStepSize
  else
    velocity_command = velocity_current - MAX_ACCEL*dynStepSize
  end
  force_command = math.min(math.abs(force_command), MAX_FORCE)
  velocity_command = math.max(velocity_command,-MAX_VELOCITY)
  velocity_command = math.min(velocity_command, MAX_VELOCITY)

--[[
  -- DEBUG
  if (not file1) then
    THOR_HOME = os.getenv('THOR_HOME')
    file1 = io.open(THOR_HOME..'/Data/logs/position'..self.joint_id..'.txt', 'w+')
    file2 = io.open(THOR_HOME..'/Data/logs/velocity'..self.joint_id..'.txt', 'w+')
    file3 = io.open(THOR_HOME..'/Data/logs/d_filter'..self.joint_id..'.txt', 'w+')
    file4 = io.open(THOR_HOME..'/Data/logs/force_cmd'..self.joint_id..'.txt', 'w+')
    file5 = io.open(THOR_HOME..'/Data/logs/velocity_cmd'..self.joint_id..'.txt', 'w+')
  else
    local _, currentVel = simGetObjectFloatParameter(jointHandle, 2012)
    file1:write(currentPos..'\n')
    file2:write(currentVel..'\n')
    file3:write(self.position_pid:get_derivative()..'\n')
    file4:write(force_command..'\n')
    file5:write(velocity_command..'\n')
  end
--]]

  return force_command, velocity_command
end

return vrep_impedance_controller
