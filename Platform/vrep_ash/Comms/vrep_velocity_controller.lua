require('pid')

local MAX_FORCE = 300
local MAX_VELOCITY = 10
local MAX_ACCEL = 600
local D_CORNER_FREQUENCY = 100

vrep_velocity_controller = {}
vrep_velocity_controller.__index = vrep_velocity_controller

function vrep_velocity_controller.new(...)
  -- Following data is handed over from V-REP:
  ------------------------------------------------------------------------------
  local init, revolute, cyclic, jointHandle, passCnt, totalPasses,
  currentPos, targetPos, errorValue, effort, dynStepSize, lowLimit,
  hightLimit, targetVel, targetForce, velUpperLimit = ...

  local o = {}
  o.joint_id = simUnpackFloats(simGetObjectCustomData(jointHandle, 2030))[1]
  return setmetatable(o, vrep_velocity_controller)
end

function vrep_velocity_controller:update(...)
  -- Following data is handed over from V-REP:
  ------------------------------------------------------------------------------
  local init, revolute, cyclic, jointHandle, passCnt, totalPasses,
  currentPos, targetPos, errorValue, effort, dynStepSize, lowLimit,
  hightLimit, targetVel, targetForce, velUpperLimit = ...

  -- Initialize velocity filter:
  ------------------------------------------------------------------------------
  if (init) then
    self.velocity_filter =
      filter.new_differentiator(dynStepSize, D_CORNER_FREQUENCY)
  end

  -- Update pid gains and setpoints:
  ------------------------------------------------------------------------------
  if (passCnt == 1) then
    local joint_param = simGetObjectCustomData(jointHandle, 2050)
    joint_param = simUnpackFloats(joint_param)
    self.force_setpoint = joint_param[1]
    self.position_setpoint = joint_param[2]
    self.velocity_setpoint = joint_param[3]
  end
 
  -- Get velocity setpoint (limit velocity to prevent overshoot):
  ------------------------------------------------------------------------------
  local velocity_command = self.velocity_setpoint
  local velocity_current = self.velocity_filter:update(currentPos)
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

return vrep_velocity_controller
