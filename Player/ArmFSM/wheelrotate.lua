local wheelrotate={}
local Body   = require'Body'
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'

-- Angular velocity limit
local dqArmMax = Config.arm.slow_limit
local dpArmMax = Config.arm.linear_slow_limit
local body_pos = {0,0,0}
local body_rpy = {0,0,0}

local lShoulderYaw = 45*Body.DEG_TO_RAD;
local rShoulderYaw = -45*Body.DEG_TO_RAD;

function wheelrotate.setArmToWheelPosition(
  handle_pos,
  handle_yaw,
  handle_pitch,
  handle_radius,
  turn_angle, 
  dt,
  lShoulderYaw,
  rShoulderYaw)

  --Calculate the hand transforms
  local trHandle = T.eye()
       * T.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * T.rotZ(handle_yaw)
       * T.rotY(handle_pitch)

  local trGripL = trHandle
       * T.rotX(turn_angle)
       * T.trans(0,handle_radius,0)
       * T.rotZ(-math.pi/4)
  local trGripR = trHandle
       * T.rotX(turn_angle)
       * T.trans(0,-handle_radius,0)
       * T.rotZ(math.pi/4)
       
  local trBody = T.eye()
       * T.trans(body_pos[1],body_pos[2],body_pos[3])
       * T.rotZ(body_rpy[3])
       * T.rotY(body_rpy[2])
       
  local trLArmTarget = T.position6D(T.inv(trBody)*trGripL)
  local trRArmTarget = T.position6D(T.inv(trBody)*trGripR)

  --Now interpolate in 6D space 
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local trLArm = Body.get_forward_larm(qLArm);
  local trRArm = Body.get_forward_rarm(qRArm);

  local trLArmApproach, doneL = util.approachTol(trLArm, trLArmTarget, dpArmMax, dt )
  local trRArmApproach, doneR = util.approachTol(trRArm, trRArmTarget, dpArmMax, dt )

  -- Get desired angles from current angles and target transform
  local qL_desired = Body.get_inverse_larm(qLArm,trLArmApproach, lShoulderYaw)
  local qR_desired = Body.get_inverse_rarm(qRArm,trRArmApproach, rShoulderYaw)
  
  if not qL_desired then
    print('Left not possible')
    return -1;
  else
--     print("qLArmDesired:",unpack(vector.new(qL_desired)*180/math.pi))
  end
  if not qR_desired then
    print('Right not possible')
    return -1;
  end

  -- Go to the allowable position
  local qL_approach, doneL2
  qL_approach, doneL2 = util.approachTolRad( qLArm, qL_desired, dqArmMax, dt )
  Body.set_larm_command_position( qL_approach )
  
  local qR_approach, doneR2
  qR_approach, doneR2 = util.approachTolRad( qRArm, qR_desired, dqArmMax, dt )
  Body.set_rarm_command_position( qR_approach )

  if doneL and doneR and doneL2 and doneR2 then
    --Approached the position
    return 1;
  else
    return 0;
  end
end

return wheelrotate