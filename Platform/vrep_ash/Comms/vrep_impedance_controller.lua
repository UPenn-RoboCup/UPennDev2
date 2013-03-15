require('pid')

--[[
console = simAuxiliaryConsoleOpen("Aux Console", 500, 0x10)
print = function(...)
  simAuxiliaryConsolePrint(console, ...)
end
--]]

local MAX_FORCE = 300
local MAX_VELOCITY = 100
local POSITION_P_GAIN_FACTOR = 1000
local POSITION_I_GAIN_FACTOR = 1000
local POSITION_D_GAIN_FACTOR = 1000
local VELOCITY_P_GAIN_FACTOR = 1000

local function sign(x)
  if (x > 0) then return 1
  elseif (x < 0) then return -1
  else return 0
  end
end 

vrep_impedance_controller = {}
vrep_impedance_controller.__index = vrep_impedance_controller

function vrep_impedance_controller.new(...)
  -- Following data is handed over from V-REP:
  ------------------------------------------------------------------------------
  local init, revolute, cyclic, jointHandle, passCnt, totalPasses,
  currentPos, targetPos, errorValue, effort, dynStepSize, lowLimit,
  hightLimit, targetVel, maxForceTorque, velUpperLimit = ...

  local o = {}
  o.position_pid = pid.new(dynStepSize)
  return setmetatable(o, vrep_impedance_controller)
end

function vrep_impedance_controller:update(...)
  -- Following data is handed over from V-REP:
  ------------------------------------------------------------------------------
  local init, revolute, cyclic, jointHandle, passCnt, totalPasses,
  currentPos, targetPos, errorValue, effort, dynStepSize, lowLimit,
  hightLimit, targetVel, maxForceTorque, velUpperLimit = ...

  local _, currentVel = simGetObjectFloatParameter(jointHandle, 2012)

  -- Initialize position pid controller:
  ------------------------------------------------------------------------------
  if (init) then
    self.position_pid:reset()
    self.position_pid:set_time_step(dynStepSize)
    self.position_pid:set_output_limits(-MAX_FORCE, MAX_FORCE)
  end

  -- Update pid gains and position setpoint:
  ------------------------------------------------------------------------------
  if (passCnt == 1) then
    local gains = simGetObjectCustomData(jointHandle, 2050)
    gains = simUnpackFloats(gains)
    local position_p_gain = gains[1] 
    local position_i_gain = gains[2]
    local position_d_gain = gains[3]
    local velocity_p_gain = gains[4]
    self.position_pid:set_gains(
      POSITION_P_GAIN_FACTOR*position_p_gain,
      POSITION_I_GAIN_FACTOR*position_i_gain,
      POSITION_D_GAIN_FACTOR*position_d_gain
    )
    self.position_pid:set_setpoint(targetPos)
  end

  -- Get velocity setpoint (limit velocity to prevent overshoot):
  ------------------------------------------------------------------------------
  -- TODO : add feedforward force
  local force_setpoint = self.position_pid:update(currentPos)
  return force_setpoint, MAX_VELOCITY*sign(force_setpoint)
end

return vrep_impedance_controller
