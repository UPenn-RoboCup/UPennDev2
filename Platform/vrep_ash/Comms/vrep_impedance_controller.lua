require('pid')

--[[
-- DEBUG
console = simAuxiliaryConsoleOpen("Aux Console", 500, 0x10)
print = function(...)
  simAuxiliaryConsolePrint(console, ...)
end
--]]

local MAX_FORCE = 300
local MAX_VELOCITY = 10
local POSITION_P_GAIN_FACTOR = 1000
local POSITION_I_GAIN_FACTOR = 1000
local POSITION_D_GAIN_FACTOR = 1000
local POSITION_D_CORNER_FREQUENCY = 250
local VELOCITY_P_GAIN_FACTOR = 1000

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

  local _, currentVel = simGetObjectFloatParameter(jointHandle, 2012)

  -- Initialize position pid controller:
  ------------------------------------------------------------------------------
  if (init) then
    self.position_pid:reset()
    self.position_pid:set_time_step(dynStepSize)
    self.position_pid:set_output_limits(-MAX_FORCE, MAX_FORCE)
    self.position_pid:set_d_corner_frequency(POSITION_D_CORNER_FREQUENCY)
  end

--[[
  -- DEBUG
  if (not file1) then
    THOR_HOME = os.getenv('THOR_HOME')
    file1 = io.open(THOR_HOME..'/Data/logs/velocity'..self.joint_id..'.txt', 'w+')
    file2 = io.open(THOR_HOME..'/Data/logs/position'..self.joint_id..'.txt', 'w+')
    file3 = io.open(THOR_HOME..'/Data/logs/d_filter'..self.joint_id..'.txt', 'w+')
  else
    file1:write(currentVel..'\n')
    file2:write(currentPos..'\n')
    file3:write(self.position_pid.d_filter.output[1]..'\n')
  end
--]]

  -- Update pid gains and position setpoint:
  ------------------------------------------------------------------------------
  if (passCnt == 1) then
    local joint_param = simGetObjectCustomData(jointHandle, 2050)
    joint_param = simUnpackFloats(joint_param)
    force_setpoint = joint_param[1]
    position_setpoint = joint_param[2]
    velocity_setpoint = joint_param[3]
    position_p_gain = POSITION_P_GAIN_FACTOR*joint_param[4]
    position_i_gain = POSITION_I_GAIN_FACTOR*joint_param[5]
    position_d_gain = POSITION_D_GAIN_FACTOR*joint_param[6]
    velocity_p_gain = VELOCITY_P_GAIN_FACTOR*joint_param[7]
    self.position_pid:set_gains(
      position_p_gain,
      position_i_gain,
      position_d_gain
    )
    self.position_pid:set_setpoint(position_setpoint)
  end

  -- Get force setpoint:
  ------------------------------------------------------------------------------
  local force_command = self.position_pid:update(currentPos)
                      + velocity_p_gain*(velocity_setpoint - currentVel) 
                      + force_setpoint
  force_command = math.min(force_command, MAX_FORCE)
  force_command = math.max(force_command,-MAX_FORCE)
  if (force_command > 0) then
    return force_command, MAX_VELOCITY
  else
    return -force_command, -MAX_VELOCITY
  end
end

return vrep_impedance_controller
