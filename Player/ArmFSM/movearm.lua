local movearm={}
local Body   = require'Body'
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'
require'hcm'

-- Angular velocity limit
local dqArmMax = Config.arm.slow_limit
local dpArmMax = Config.arm.linear_slow_limit
local body_pos = {0,0,0}
local body_rpy = {0,0,0}

local lShoulderYaw = 45*Body.DEG_TO_RAD;
local rShoulderYaw = -45*Body.DEG_TO_RAD;

function movearm.setArmJoints(
  qLArmTarget,
  qRArmTarget,
  dt,
  dqArmLim
  )
  if not dqArmLim then dqArmLim = dqArmMax; end
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local qL_approach, doneL2
  qL_approach, doneL2 = util.approachTolRad( qLArm, qLArmTarget, dqArmLim, dt )
  Body.set_larm_command_position( qL_approach )
  
  local qR_approach, doneR2
  qR_approach, doneR2 = util.approachTolRad( qRArm, qRArmTarget, dqArmLim, dt )
  Body.set_rarm_command_position( qR_approach )
  if doneL2 and doneR2 then 
    return 1;
  end
end

function movearm.setArmToPosition(
  trLArmTarget,
  trRArmTarget,
  dt,
  lShoulderYaw,
  rShoulderYaw
  )

  
  --Interpolate in 6D space 
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()

  lShoulderYaw = qLArm[3]
  rShoulderYaw = qRArm[3]


--  lShoulderYaw = hcm.get_joints_qlshoulderyaw()
--  rShoulderYaw = hcm.get_joints_qrshoulderyaw()

  local trLArm = Body.get_forward_larm(qLArm);
  local trRArm = Body.get_forward_rarm(qRArm);

  local trLArmApproach, doneL = util.approachTol(trLArm, trLArmTarget, dpArmMax, dt )
  local trRArmApproach, doneR = util.approachTol(trRArm, trRArmTarget, dpArmMax, dt )

  -- Get desired angles from current angles and target transform
  local qL_desired = Body.get_inverse_larm(qLArm,trLArmApproach, lShoulderYaw)
  local qR_desired = Body.get_inverse_rarm(qRArm,trRArmApproach, rShoulderYaw)
  

  qShoulderYawMax = 3.0*math.pi/180
  --Adaptive shoulder yaw angle
  local lShoulderYaw1 = math.min(0, lShoulderYaw+dt*qShoulderYawMax)
  local lShoulderYaw2 = math.min(0, lShoulderYaw-dt*qShoulderYawMax)
  local qL_desired1 = Body.get_inverse_larm(qLArm,trLArmApproach, lShoulderYaw1)
  local qL_desired2 = Body.get_inverse_larm(qLArm,trLArmApproach, lShoulderYaw2)

  local rShoulderYaw1 = math.max(0,rShoulderYaw+dt*qShoulderYawMax)
  local rShoulderYaw2 = math.max(0,rShoulderYaw-dt*qShoulderYawMax)
  local qR_desired1 = Body.get_inverse_rarm(qRArm,trRArmApproach, rShoulderYaw1)
  local qR_desired2 = Body.get_inverse_rarm(qRArm,trRArmApproach, rShoulderYaw2)

  local min_wrist_margin = 10*math.pi/180

  local min_wrist_margin0 = 3*math.pi/180


  if qL_desired and qR_desired then
    --Check left wrist margin status
    if qL_desired1 then
      local wristRollMargin=math.min(math.abs(qL_desired[6]-math.pi/2), math.abs(qL_desired[6]+math.pi/2))
      local wristRollMargin1=math.min(math.abs(qL_desired1[6]-math.pi/2), math.abs(qL_desired1[6]+math.pi/2))
      local wristRollMargin2=0
      if qL_desired2 then 
        wristRollMargin2=math.min(math.abs(qL_desired2[6]-math.pi/2), math.abs(qL_desired2[6]+math.pi/2))
      end
      if wristRollMargin<min_wrist_margin then
        if wristRollMargin2>wristRollMargin and wristRollMargin2>wristRollMargin1 then
          qL_desired = qL_desired2;
--        hcm.set_joints_qlshoulderyaw(lShoulderYaw2)
          print("LSW:",lShoulderYaw2*180/math.pi)
        elseif wristRollMargin1>wristRollMargin and wristRollMargin1>wristRollMargin2 then       
          qL_desired = qL_desired1;
--        hcm.set_joints_qlshoulderyaw(lShoulderYaw1)
          print("LSW:",lShoulderYaw2*180/math.pi)
        end
      end
    end

    --Check right wrist margin status
    if qR_desired1 then
      local wristRollMargin=math.min(math.abs(qR_desired[6]-math.pi/2), math.abs(qR_desired[6]+math.pi/2))
      local wristRollMargin1=math.min(math.abs(qR_desired1[6]-math.pi/2), math.abs(qR_desired1[6]+math.pi/2))
      local wristRollMargin2=0
      if qR_desired2 then 
        wristRollMargin2=math.min(math.abs(qR_desired2[6]-math.pi/2), math.abs(qR_desired2[6]+math.pi/2))
      end
      if wristRollMargin<min_wrist_margin then
        if wristRollMargin2>wristRollMargin and wristRollMargin2>wristRollMargin1 then
          qR_desired = qR_desired2;
          hcm.set_joints_qrshoulderyaw(rShoulderYaw2)
          print("RSW:",rShoulderYaw2*180/math.pi)
        elseif wristRollMargin1>wristRollMargin and wristRollMargin1>wristRollMargin2 then       
          qR_desired = qR_desired1;
          hcm.set_joints_qrshoulderyaw(rShoulderYaw1)
          print("RSW:",rShoulderYaw1*180/math.pi)
        end
      end
    end

  else
    if not qL_desired then
      print('Left not possible')
    end  
    if not qR_desired then
      print('Right not possible')
    end
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





function movearm.setArmToWheelPosition(
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

  return movearm.setArmToPosition(
    trLArmTarget,
    trRArmTarget,
    dt,
    lShoulderYaw,
    rShoulderYaw)
end

return movearm