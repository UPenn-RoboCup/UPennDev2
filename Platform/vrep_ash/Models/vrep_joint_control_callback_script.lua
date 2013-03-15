-- copy paste this code into the vrep custom control loop for each joint

local init, revolute, cyclic, jointHandle, passCnt, totalPasses,
currentPos, targetPos, errorValue, effort, dynStepSize, lowLimit,
hightLimit, targetVel, maxForceTorque, velUpperLimit = ...

if (init) then
  THOR_HOME = os.getenv('THOR_HOME')
  dofile(THOR_HOME..'/Run/include.lua')
  local controller = simUnpackFloats(simGetObjectCustomData(jointHandle, 2040))
  if (controller[1] == 0) then
    controller = require('vrep_impedance_controller')
  elseif (controller[1] == 1) then
    controller = require('vrep_position_controller')
  elseif (controller[1] == 2) then
    controller = require('vrep_velocity_controller')
  end
  joint_controller = controller.new(...)
end

return joint_controller:update(...)
