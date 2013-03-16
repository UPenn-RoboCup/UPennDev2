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
 
  -- Get velocity setpoint (limit velocity to prevent overshoot):
  ------------------------------------------------------------------------------
  local velocity_command = self.position_pid:update(currentPos)
  velocity_command = math.min(velocity_command, math.abs(errorValue/dynStepSize))
  velocity_command = math.max(velocity_command,-math.abs(errorValue/dynStepSize))
  return MAX_FORCE, velocity_command
end

return vrep_position_controller
