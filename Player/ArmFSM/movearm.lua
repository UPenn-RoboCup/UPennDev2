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

local lShoulderYaw = -45*Body.DEG_TO_RAD;
local rShoulderYaw = 45*Body.DEG_TO_RAD;

function movearm.setArmJoints(
  qLArmTarget,
  qRArmTarget,
  dt,
  dqArmLim
  )
  if not dqArmLim then dqArmLim = dqArmMax end
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  local qL_approach, doneL2
  qL_approach, doneL2 = util.approachTolRad( qLArm, qLArmTarget, dqArmLim, dt )
  
  local qR_approach, doneR2
  qR_approach, doneR2 = util.approachTolRad( qRArm, qRArmTarget, dqArmLim, dt )

  local qL = Body.set_larm_command_position( qL_approach )
  local qR = Body.set_rarm_command_position( qR_approach )

  -- What if out of bounds for the joint angles?
  -- Then, return a filtered target position (qL[i]~=qL_approach)
  -- yields the index of the joint that is out-of-bounds
  -- We should reset this particular joint in hcm, so the user cannot
  -- move the joint into no man's land

  -- When done, return true
  if doneL2 and doneR2 then return 1 end
end


--Manual set of shoulder yaw angle
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
  local yawDoneL = true;
  local yawDoneR = true;

  if not lShoulderYaw then
    lShoulderYaw = qLArm[3]
    rShoulderYaw = qRArm[3]    
  else
    --Change arm yaw to target agle 
    qShoulderYawMax = 5.0*math.pi/180
    lShoulderYaw0 = qLArm[3]
    rShoulderYaw0 = qRArm[3]        
    lShoulderYaw, yawDoneL = util.approachTol(lShoulderYaw0,lShoulderYaw,qShoulderYawMax,dt)
    rShoulderYaw, yawDoneR = util.approachTol(rShoulderYaw0,rShoulderYaw,qShoulderYawMax,dt)
  end

  local trLArm = Body.get_forward_larm(qLArm);
  local trRArm = Body.get_forward_rarm(qRArm);

  local trLArmApproach, doneL = util.approachTol(trLArm, trLArmTarget, dpArmMax, dt )
  local trRArmApproach, doneR = util.approachTol(trRArm, trRArmTarget, dpArmMax, dt )

  -- Get desired angles from current angles and target transform
  local qL_desired = Body.get_inverse_larm(qLArm,trLArmApproach, lShoulderYaw)
  local qR_desired = Body.get_inverse_rarm(qRArm,trRArmApproach, rShoulderYaw)
  
  if qL_desired and qR_desired then
    -- Go to the allowable position
    local qL_approach, doneL2
    qL_approach, doneL2 = util.approachTolRad( qLArm, qL_desired, dqArmMax, dt )
    Body.set_larm_command_position( qL_approach )
  
    local qR_approach, doneR2
    qR_approach, doneR2 = util.approachTolRad( qRArm, qR_desired, dqArmMax, dt )
    Body.set_rarm_command_position( qR_approach )

    if doneL and doneR and doneL2 and doneR2 and yawDoneL and yawDoneR then
      --Approached the position
      return 1;
    else
      return 0;
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
end


function movearm.setWristPosition(
  trLWristTarget,
  trRWristTarget,
  dt,
  lShoulderYaw,
  rShoulderYaw 
  )
  
  --Interpolate in 6D space 
  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()
  if not lShoulderYaw then
    lShoulderYaw = qLArm[3]
    rShoulderYaw = qRArm[3]    
  end

  qShoulderYawMax = 5.0*math.pi/180
  lShoulderYaw = util.approachTol(qLArm[3],lShoulderYaw,qShoulderYawMax,dt)
  rShoulderYaw = util.approachTol(qRArm[3],rShoulderYaw,qShoulderYawMax,dt)

  local trLWrist = Body.get_forward_lwrist(qLArm);
  local trRWrist = Body.get_forward_rwrist(qRArm);
  
  local trLWristApproach, doneL = util.approachTol(trLWrist, trLWristTarget, dpArmMax, dt )
  local trRWristApproach, doneR = util.approachTol(trRWrist, trRWristTarget, dpArmMax, dt )

  -- Get desired angles from current angles and target transform
  local qL_desired = Body.get_inverse_lwrist(qLArm,trLWristApproach, lShoulderYaw)
  local qR_desired = Body.get_inverse_rwrist(qRArm,trRWristApproach, rShoulderYaw)  
  
  if qL_desired and qR_desired then
    local qL_approach, doneL
    qL_approach, doneL = util.approachTolRad( qLArm, qL_desired, dqArmMax, dt )
    Body.set_larm_command_position( qL_approach )
  
    local qR_approach, doneR
    qR_approach, doneR = util.approachTolRad( qRArm, qR_desired, dqArmMax, dt )
    Body.set_rarm_command_position( qR_approach )
    if doneL and doneR then
      return 1;
    else      
      return 0;
    end
  else    
    print("ERROR WITH WRIST IK")
    return -1;
  end  
end













function movearm.setArmToPositionAdapt(
  trLArmTarget,
  trRArmTarget,
  dt
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
  

  qShoulderYawMax = 5.0*math.pi/180
  --Adaptive shoulder yaw angle
  lShoulderYawOrg = -24*math.pi/180;
  rShoulderYawOrg = 24*math.pi/180;

  
  local lShoulderYaw1 = math.min(-5*math.pi/180, lShoulderYaw+dt*qShoulderYawMax )
  local lShoulderYaw2 = math.min(-5*math.pi/180, lShoulderYaw-dt*qShoulderYawMax )
  local qL_desired1 = Body.get_inverse_larm(qLArm,trLArmApproach, lShoulderYaw1)
  local qL_desired2 = Body.get_inverse_larm(qLArm,trLArmApproach, lShoulderYaw2)

  local rShoulderYaw1 = math.max(5*math.pi/180,rShoulderYaw+dt*qShoulderYawMax)
  local rShoulderYaw2 = math.max(5*math.pi/180,rShoulderYaw-dt*qShoulderYawMax)
  local qR_desired1 = Body.get_inverse_rarm(qRArm,trRArmApproach, rShoulderYaw1)
  local qR_desired2 = Body.get_inverse_rarm(qRArm,trRArmApproach, rShoulderYaw2)

--[[
  --SJ: slightly harder check
  --If any of the candidates fail, it returns fail
  if not qL_desired or not qL_desired1 or not qL_desired2 then
    print('Left not possible')
    return -1;
  end
  if not qR_desired or not qR_desired1 or not qR_desired2 then
    print('Right not possible')
    return -1;
  end
 --]]



  --Adjust shoulder angle if margin is less than this
  local min_wrist_margin = 10*math.pi/180

  --Return shoulder angle if margin is larger than this
  local min_wrist_margin1 = 15*math.pi/180


  if qL_desired and qR_desired then
    local shoulderYawChanged = false;
    --Check left wrist margin status
    if qL_desired1 then
      local wristRollMargin=math.min(
        math.abs(qL_desired[6]-math.pi/2),
        math.abs(qL_desired[6]+math.pi/2),
        math.abs(qL_desired[2]-math.pi/2),
        math.abs(qL_desired[2])
        )
      local wristRollMargin1=math.min(
        math.abs(qL_desired1[6]-math.pi/2),
        math.abs(qL_desired1[6]+math.pi/2),
        math.abs(qL_desired1[2]-math.pi/2),
        math.abs(qL_desired1[2])
        )
      local wristRollMargin2=-math.huge
      if qL_desired2 then 
        wristRollMargin2=math.min(
        math.abs(qL_desired2[6]-math.pi/2),
        math.abs(qL_desired2[6]+math.pi/2),
        math.abs(qL_desired2[2]-math.pi/2),
        math.abs(qL_desired2[2])
        )
      end
      if wristRollMargin<min_wrist_margin then
        if wristRollMargin2>wristRollMargin and wristRollMargin2>wristRollMargin1 then
          qL_desired = qL_desired2
          lShoulderYaw = lShoulderYaw2
          shoulderYawChanged = true;
        elseif wristRollMargin1>wristRollMargin and wristRollMargin1>wristRollMargin2 then       
          qL_desired = qL_desired1;
          lShoulderYaw = lShoulderYaw1          
          shoulderYawChanged = true;
        end
      elseif wristRollMargin>min_wrist_margin1 then
        if math.abs(lShoulderYawOrg-lShoulderYaw1)<math.abs(lShoulderYawOrg-lShoulderYaw2) and
           math.abs(lShoulderYawOrg-lShoulderYaw1)<math.abs(lShoulderYawOrg-lShoulderYaw) then
          qL_desired = qL_desired1;
          lShoulderYaw = lShoulderYaw1          
          shoulderYawChanged = true;
        elseif math.abs(lShoulderYawOrg-lShoulderYaw2)<math.abs(lShoulderYawOrg-lShoulderYaw1) and
           math.abs(lShoulderYawOrg-lShoulderYaw2)<math.abs(lShoulderYawOrg-lShoulderYaw) then
          qL_desired = qL_desired2;
          lShoulderYaw = lShoulderYaw2          
          shoulderYawChanged = true;
        end
      end
    end

    --Check right wrist margin status
    if qR_desired1 then
      local wristRollMargin=math.min(
        math.abs(qR_desired[6]-math.pi/2),
        math.abs(qR_desired[6]+math.pi/2),
        math.abs(qR_desired[2]+math.pi/2),
        math.abs(qR_desired[2])
        )
      local wristRollMargin1=math.min(
        math.abs(qR_desired1[6]-math.pi/2),
        math.abs(qR_desired1[6]+math.pi/2),
        math.abs(qR_desired1[2]+math.pi/2),
        math.abs(qR_desired1[2])
        )
      local wristRollMargin2=-math.huge
      if qR_desired2 then 
        wristRollMargin2=math.min(
        math.abs(qR_desired2[6]-math.pi/2),
        math.abs(qR_desired2[6]+math.pi/2),
        math.abs(qR_desired2[2]+math.pi/2),
        math.abs(qR_desired2[2])
        )
      end
      if wristRollMargin<min_wrist_margin then
        if wristRollMargin2>wristRollMargin and wristRollMargin2>wristRollMargin1 then
          qR_desired = qR_desired2;
          hcm.set_joints_qrshoulderyaw(rShoulderYaw2)
          shoulderYawChanged = true;
        elseif wristRollMargin1>wristRollMargin and wristRollMargin1>wristRollMargin2 then       
          qR_desired = qR_desired1;
          hcm.set_joints_qrshoulderyaw(rShoulderYaw1)
          shoulderYawChanged = true;
        end
      elseif wristRollMargin>min_wrist_margin1 then
        if math.abs(rShoulderYawOrg-rShoulderYaw1)<math.abs(rShoulderYawOrg-rShoulderYaw2) and
           math.abs(rShoulderYawOrg-rShoulderYaw1)<math.abs(rShoulderYawOrg-rShoulderYaw) then
          qR_desired = qR_desired1;
          rShoulderYaw = rShoulderYaw1          
          shoulderYawChanged = true;
        elseif math.abs(rShoulderYawOrg-rShoulderYaw2)<math.abs(rShoulderYawOrg-rShoulderYaw1) and
           math.abs(rShoulderYawOrg-rShoulderYaw2)<math.abs(rShoulderYawOrg-rShoulderYaw) then
          qR_desired = qR_desired2;
          rShoulderYaw = rShoulderYaw2          
          shoulderYawChanged = true;
        end
      end
    end

    if shoulderYawChanged then
--      print("ShoulderYaw:",lShoulderYaw*180/math.pi,rShoulderYaw*180/math.pi)          
    end
  end

  if not qL_desired or not qR_desired then
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

--  if doneL and doneR and doneL2 and doneR2 and not shoulderYawChanged then
  if doneL and doneR and doneL2 and doneR2 then
    --Approached the position
    return 1;
  else
    return 0;
  end

  

  hcm.set_joints_qlshoulderyaw(lShoulderYaw)
  hcm.set_joints_qrshoulderyaw(rShoulderYaw)

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

  if lShoulderYaw then
    return movearm.setArmToPosition(
      trLArmTarget,
      trRArmTarget,
      dt,
      lShoulderYaw,
      rShoulderYaw)
  else
    return movearm.setArmToPositionAdapt(
      trLArmTarget,
      trRArmTarget,
      dt)
  end
end

function movearm.getDoorHandlePosition(
  hinge_pos, 
  door_r, --distance between hinge and the projection of grip position on the door
          --positive value for right-hinged door, negative value for left-hinged door
  door_yaw, --door angle, -pi/2~pi/2
  grip_offset_x, --how much the actual grip positon offset from door surface
  hand_rpy
  )
  --Default hand angle: facing up
  hand_rpy = hand_rpy or {-90*Body.DEG_TO_RAD,0,0}
    
  local trHandle = T.eye()
    * T.trans(hinge_pos[1],hinge_pos[2],hinge_pos[3])
    * T.rotZ(door_yaw)
    * T.trans(grip_offset_x, door_r, 0) 
    * T.transform6D(
      {0,0,0,hand_rpy[1],hand_rpy[2],hand_rpy[3]})  


  local trBody = T.eye()
       * T.trans(body_pos[1],body_pos[2],body_pos[3])
       * T.rotZ(body_rpy[3])
       * T.rotY(body_rpy[2])

  local trTarget = T.position6D(T.inv(trBody)*trHandle)
  return trTarget
end

return movearm