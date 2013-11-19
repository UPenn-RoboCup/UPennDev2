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

function movearm.setArmJoints(qLArmTarget,qRArmTarget, dt,dqArmLim)
  if not dqArmLim then dqArmLim = dqArmMax end

  local qLArm = Body.get_larm_command_position()
  local qRArm = Body.get_rarm_command_position()  
 
  local qL_approach, doneL2 = util.approachTolRad( qLArm, qLArmTarget, dqArmLim, dt )  
  local qR_approach, doneR2 = util.approachTolRad( qRArm, qRArmTarget, dqArmLim, dt )

  --Dynamixel is STUPID so we should manually check for the direction
  for i=1,7 do
    local qL_increment = util.mod_angle(qL_approach[i]-qLArm[i])
    local qR_increment = util.mod_angle(qR_approach[i]-qRArm[i])
    qL_approach[i] = qLArm[i] + qL_increment
    qR_approach[i] = qRArm[i] + qR_increment
  end

  Body.set_larm_command_position( qL_approach )
  Body.set_rarm_command_position( qR_approach )
  if doneL2 and doneR2 then return 1 end
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
  velWristMax = Config.arm.linear_wrist_limit
  lShoulderYaw = util.approachTol(qLArm[3],lShoulderYaw,qShoulderYawMax,dt)
  rShoulderYaw = util.approachTol(qRArm[3],rShoulderYaw,qShoulderYawMax,dt)

  local trLWrist = Body.get_forward_lwrist(qLArm);
  local trRWrist = Body.get_forward_rwrist(qRArm);
  
  local trLWristApproach, doneL = util.approachTol(trLWrist, trLWristTarget, dpArmMax, dt )
  local trRWristApproach, doneR = util.approachTol(trRWrist, trRWristTarget, dpArmMax, dt )

--[[
  local trLWristApproach, doneL = util.approachTolTransform(
        trLWrist, trLWristTarget, velWristMax, dt )
  local trRWristApproach, doneR = util.approachTolTransform(
        trRWrist, trRWristTarget, velWristMax, dt )
--]]


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
    --Adaptive movement
    lShoulderYaw = qLArm[3]
    rShoulderYaw = qRArm[3]    
  else
    --Manual yaw movement
    --Change arm yaw to target agle 
--    qShoulderYawMax = 5.0*math.pi/180

    qShoulderYawMax = 10.0*math.pi/180

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


function movearm.get_jointangle_margin(
  arm, qArm)
  local jointangle_margin
  if not qArm then --Joint angle out of bounds
    return -math.huge 
  elseif arm==1 then --Left arm
    jointangle_margin = math.min(
      math.abs(qArm[6]-math.pi/2),
      math.abs(qArm[6]+math.pi/2),
      math.abs(qArm[2]-math.pi/2),
      math.abs(qArm[2])
      )
  else --Right arm
    jointangle_margin = math.min(
      math.abs(qArm[6]-math.pi/2),
      math.abs(qArm[6]+math.pi/2),
      math.abs(qArm[2]+math.pi/2),
      math.abs(qArm[2]) 
      )
  end
  return jointangle_margin
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

  local trLArm = Body.get_forward_larm(qLArm);
  local trRArm = Body.get_forward_rarm(qRArm);

  local trLArmApproach, doneL = util.approachTolTransform(trLArm, trLArmTarget, dpArmMax, dt )
  local trRArmApproach, doneR = util.approachTolTransform(trRArm, trRArmTarget, dpArmMax, dt )

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
    local wristRollMargin = movearm.get_jointangle_margin(1, qL_desired)
    local wristRollMargin1 = movearm.get_jointangle_margin(1, qL_desired1)
    local wristRollMargin2 = movearm.get_jointangle_margin(1, qL_desired2)            

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

    --Check right wrist margin status
    local wristRollMargin = movearm.get_jointangle_margin(0, qR_desired)
    local wristRollMargin1 = movearm.get_jointangle_margin(0, qR_desired1)
    local wristRollMargin2 = movearm.get_jointangle_margin(0, qR_desired2)            

    if wristRollMargin<min_wrist_margin then
      if wristRollMargin2>wristRollMargin and wristRollMargin2>wristRollMargin1 then
        qR_desired = qR_desired2;
        shoulderYawChanged = true;
      elseif wristRollMargin1>wristRollMargin and wristRollMargin1>wristRollMargin2 then       
        qR_desired = qR_desired1;
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
    if shoulderYawChanged then
--      print("ShoulderYaw:",lShoulderYaw*180/math.pi,rShoulderYaw*180/math.pi)          
    end
  end

  if not qL_desired or not qR_desired then
    if not qL_desired then print('Left not possible')  end  
    if not qR_desired then print('Right not possible') end
    return -1;
  end

  -- Go to the allowable position
  local qL_approach, doneL2
  qL_approach, doneL2 = util.approachTolRad( qLArm, qL_desired, dqArmMax, dt )
  Body.set_larm_command_position( qL_approach )
  
  local qR_approach, doneR2
  qR_approach, doneR2 = util.approachTolRad( qRArm, qR_desired, dqArmMax, dt )
  Body.set_rarm_command_position( qR_approach )

  if doneL and doneR and doneL2 and doneR2 then --Approached the position
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
  local trLArmTarget, trRArmTarget = movearm.getArmWheelPosition(
    handle_pos,
    handle_yaw,
    handle_pitch,
    handle_radius,
    turn_angle
    )
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




function movearm.getArmWheelPosition(handle_pos, 
     handle_yaw,  handle_pitch,  handle_radius,  turn_angle  )

  --Calculate the hand transforms
  local trHandle = T.eye()
       * T.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * T.rotZ(handle_yaw)
       * T.rotY(handle_pitch)
  local gripOffset = 0*math.pi/180
  local trGripL = trHandle
       * T.rotX(turn_angle + gripOffset)
       * T.trans(0,handle_radius,0)
       * T.rotZ(-math.pi/4)
  local trGripR = trHandle
       * T.rotX(-turn_angle - gripOffset)
       * T.trans(0,-handle_radius,0)
       * T.rotZ(math.pi/4)
  local trBody = T.eye()
       * T.trans(body_pos[1],body_pos[2],body_pos[3])
       * T.rotZ(body_rpy[3])
       * T.rotY(body_rpy[2])
  local trLArmTarget = T.position6D(T.inv(trBody)*trGripL)
  local trRArmTarget = T.position6D(T.inv(trBody)*trGripR)
  return trLArmTarget, trRArmTarget
end

function movearm.getDoorHandlePosition(
  pos_offset,
  knob_roll,
  door_yaw
  )
  local door_model = hcm.get_door_model()  
  local hinge_pos = vector.slice(door_model,1,3) + vector.new(pos_offset)  
  local door_r = door_model[4]
  local grip_offset_x = door_model[5]
  local knob_offset_y = door_model[6]
 
  local rhand_rpy0 = {-90*Body.DEG_TO_RAD,-5*Body.DEG_TO_RAD,0}

--  local rhand_rpy0 = {-90*Body.DEG_TO_RAD,0*Body.DEG_TO_RAD,0}

  local hand_rpy = rhand_rpy0

  local hand_yaw = door_yaw
  if door_yaw>10*Body.DEG_TO_RAD then
    hand_yaw = door_yaw-(door_yaw-10*Body.DEG_TO_RAD)*2.5 
  end


--[[
  local hand_yaw = 0
  if door_yaw>10*Body.DEG_TO_RAD then
    hand_yaw = -(door_yaw-10*Body.DEG_TO_RAD)*1.5 
  end
--]]

  local trHandle = T.eye()
    * T.trans(hinge_pos[1],hinge_pos[2],hinge_pos[3])    
    * T.rotZ(door_yaw)
    * T.trans(grip_offset_x, door_r+knob_offset_y, 0)     
    * T.rotX(knob_roll)
    * T.trans(0,-knob_offset_y, 0) 
    * T.rotX(-knob_roll)
    * T.rotZ(hand_yaw-door_yaw)
    * T.rotX(knob_roll)
    * T.transform6D(
      {0,0,0,hand_rpy[1],hand_rpy[2],hand_rpy[3]})  

  local trTarget = T.position6D(trHandle)
  return trTarget
end




















--Use two chopstick hand
function movearm.getLargeValvePosition(
  turn_angleL,
  turn_angleR,
  offsetL,
  offsetR
  )

  local wheel   = hcm.get_wheel_model()
  local handle_pos    = vector.slice(wheel,1,3)
  local handle_yaw    = wheel[4]
  local handle_pitch  = 0 --we assume zero pitch 
  local handle_radius = wheel[6] 

  local lhand_rpy = {0,0*Body.DEG_TO_RAD, -10*Body.DEG_TO_RAD}
  local rhand_rpy = {0,0*Body.DEG_TO_RAD, 10*Body.DEG_TO_RAD}

  --Calculate the hand transforms
  local trHandle = T.eye()
       * T.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * T.rotZ(handle_yaw)       
  
  local trGripL = trHandle
       * T.rotX(turn_angleL)
       * T.trans(0,handle_radius,0)
       * T.transform6D(
          {0,0,0,lhand_rpy[1],lhand_rpy[2],lhand_rpy[3]})  
       * T.trans(offsetL,0,0)

  local trGripR = trHandle
       * T.rotX(turn_angleR)
       * T.trans(0,-handle_radius,0)
       * T.transform6D(
          {0,0,0,rhand_rpy[1],rhand_rpy[2],rhand_rpy[3]})  
       * T.trans(offsetR,0,0)       
  return T.position6D(trGripL), T.position6D(trGripR)
end

--Use LEFT chopstick hand
function movearm.getLargeValvePositionSingle(
  turn_angle,
  offset,
  is_left
  )

  local wheel = hcm.get_largevalve_model()
  local handle_pos = vector.slice(wheel,1,3)

  --we assume zero yaw and pitch 
  local handle_yaw    = 0
  local handle_pitch  = 0 
  local handle_radius = wheel[4] 
  local hand_rpy = {0,0*Body.DEG_TO_RAD, 0*Body.DEG_TO_RAD}

  --Calculate the hand transforms
  local trHandle = T.eye()
       * T.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * T.rotZ(handle_yaw)       
  if is_left>0 then
    local trGripL = trHandle
       * T.rotX(turn_angle)
       * T.trans(0,handle_radius,0)
       * T.transform6D(
          {0,0,0,hand_rpy[1],hand_rpy[2],hand_rpy[3]})  
       * T.trans(offset,0,0)
    return T.position6D(trGripL)
  else
    local trGripR = trHandle
       * T.rotX(turn_angle)
       * T.trans(0,-handle_radius,0)
       * T.transform6D(
          {0,0,0,hand_rpy[1],hand_rpy[2],hand_rpy[3]})  
       * T.trans(offset,0,0) 
     return T.position6D(trGripR)      
  end
end


--Use LEFT chopstick hand
function movearm.getBarValvePositionSingle(
  turn_angle,
  wrist_angle,
  offset
  )

  local wheel = hcm.get_barvalve_model()
  local handle_pos = vector.slice(wheel,1,3)
  local handle_radius = wheel[4]


  --we assume zero yaw and pitch 
  local handle_yaw    = 0
  local handle_pitch  = 0 
  
  --We assume verticle, downward valve as zero turnangle (and vertial chopsticks)
  local hand_rpy = {0,0*Body.DEG_TO_RAD, 0*Body.DEG_TO_RAD}

  --Calculate the hand transforms
  local trHandle = T.eye()
       * T.trans(handle_pos[1],handle_pos[2],handle_pos[3])
       * T.rotZ(handle_yaw)       
  
  local trGripL = trHandle
       * T.rotX(turn_angle)
       * T.trans(0,0,-handle_radius)
       * T.rotX(wrist_angle)
       * T.transform6D(
          {0,0,0,hand_rpy[1],hand_rpy[2],hand_rpy[3]})  
       * T.trans(offset,0,0)
  return T.position6D(trGripL)
  
end


return movearm