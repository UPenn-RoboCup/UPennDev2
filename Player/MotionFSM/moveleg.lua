local moveleg={}
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'

-- SJ: Shared library for 2D leg trajectory generation
-- So that we can reuse them for different controllers
-- Should we move it into util?



-- Velocity limits used in update_velocity function
local velLimitX = Config.walk.velLimitX or {-.06, .08}
local velLimitY = Config.walk.velLimitY or {-.06, .06}
local velLimitA = Config.walk.velLimitA or {-.4, .4}
local velDelta  = Config.walk.velDelta or {.03,.015,.15}
local vaFactor  = Config.walk.vaFactor or 0.6

-- Foothold Generation parameters ---------------------------------------
local stanceLimitX = Config.walk.stanceLimitX or {-0.10 , 0.10}
local stanceLimitY = Config.walk.stanceLimitY or {0.09 , 0.20}
local stanceLimitA = Config.walk.stanceLimitA or {-0*math.pi/180, 40*math.pi/180}

-- Toe/heel overlap checking values
local footSizeX = Config.walk.footSizeX or {-0.05,0.05}
local stanceLimitMarginY = Config.walk.stanceLimitMarginY or 0.015
local stanceLimitY2 = 2* Config.walk.footY-stanceLimitMarginY
-------------------------------------------------------------------------

local footY    = Config.walk.footY
local supportX = Config.walk.supportX
local supportY = Config.walk.supportY
local torsoX    = Config.walk.torsoX

-- Gyro stabilization parameters
local ankleImuParamX = Config.walk.ankleImuParamX
local ankleImuParamY = Config.walk.ankleImuParamY
local kneeImuParamX  = Config.walk.kneeImuParamX
local hipImuParamY   = Config.walk.hipImuParamY

-- Hip sag compensation parameters
local hipRollCompensation = Config.walk.hipRollCompensation

-- Leg hardness parameters
local hardnessSupport = Config.walk.hardnessSupport or 0.7
local hardnessSwing   = Config.walk.hardnessSwing or 0.5


function moveleg.update_velocity(velCurrent)
  -- Grab from the shared memory the desired walking speed
--  local vx,vy,va = unpack(mcm.get_walk_vel())
  local vx,vy,va = unpack(hcm.get_motion_velocity())

  --Filter the commanded speed
  vx = math.min(math.max(vx,velLimitX[1]),velLimitX[2])
  vy = math.min(math.max(vy,velLimitY[1]),velLimitY[2])
  va = math.min(math.max(va,velLimitA[1]),velLimitA[2])
  -- Find the magnitude of the velocity
  local stepMag = math.sqrt(vx^2+vy^2)
  --Slow down when turning
  local vFactor   = 1-math.abs(va)/vaFactor
  local magFactor = math.min(velLimitX[2]*vFactor,stepMag)/(stepMag+0.000001)
  -- Limit the forwards and backwards velocity
  vx = math.min(math.max(vx*magFactor,velLimitX[1]),velLimitX[2])
  vy = math.min(math.max(vy*magFactor,velLimitY[1]),velLimitY[2])
  va = math.min(math.max(va,velLimitA[1]),velLimitA[2])
  -- Check the change in the velocity
  local velDiff = vector.new({vx,vy,va}) - velCurrent
  -- Limit the maximum velocity change PER STEP
  velDiff[1] = util.procFunc(velDiff[1],0,velDelta[1])
  velDiff[2] = util.procFunc(velDiff[2],0,velDelta[2])
  velDiff[3] = util.procFunc(velDiff[3],0,velDelta[3])

  -- Update the current velocity command
  return velCurrent + velDiff
end

function moveleg.advance_step(uLeft, uRight,uTorso,iStep,velCurrent,enable_initial_step)
  

  supportLeg = iStep % 2 -- supportLeg: 0 for left support, 1 for right support
  --Zero velocity at the initial steps
  -- iStep:2 for 1st step, 3 for 2st step

  if not enable_initial_step then iStep = 5 end --don't do initial step thing
  if iStep<=3 then velCurrent = vector.new{0,0,0} end
  local uLeft_now, uRight_now, uTorso_now = uLeft, uRight, uTorso
  local uSupport, uLeft_next, uRight_next = 
    moveleg.calculate_next_step( uLeft, uRight, supportLeg, velCurrent )
  if iStep<=3  then uLeft_next = uLeft_now;uRight_next = uRight_now;end

  -- This is the next desired torso position    
  uTorso_next = moveleg.calculate_next_torso( uLeft_next, uRight_next, supportLeg, 0.5 )
  if enable_intial_step and iStep<=3 then
    --support should be closer to center for initial step
    uSupport = util.se2_interpolate(0.3,uSupport,uTorso_next)      
  end  
  -- Save some step-by-step data to shared memory
  mcm.set_status_velocity(velCurrent)
  mcm.set_support_uLeft_now(  uLeft_now )
  mcm.set_support_uRight_now( uRight_now )
  mcm.set_support_uTorso_now( uTorso_now )
  mcm.set_support_uLeft_next(  uLeft_next )
  mcm.set_support_uRight_next( uRight_next )
  mcm.set_support_uTorso_next( uTorso_next )

  return uLeft_now,uRight_now,uTorso_now, uLeft_next,uRight_next,uTorso_next, uSupport
end


function moveleg.calculate_next_step(uLeft_now, uRight_now, supportLeg, uBody_diff)
  if supportLeg == 0 then    -- Left support
    -- Find the left support point
    local uSupport = util.pose_global({supportX, supportY, 0}, uLeft_now)
    -- Find the right foot destination
    local uRight_next = moveleg.step_destination_right(uBody_diff, uLeft_now, uRight_now)
    return uSupport, uLeft_now, uRight_next
  else    -- Right support
    -- Find the right support point
    local uSupport = util.pose_global({supportX, -supportY, 0}, uRight_now)
    -- Find the left foot destination
    local uLeft_next = moveleg.step_destination_left(uBody_diff, uLeft_now, uRight_now)
    return uSupport, uLeft_next, uRight_now
  end
end

function moveleg.calculate_next_torso(uLeft_next,uRight_next,supportLeg, shiftFactor)
  -- shiftFactor: How much should we shift final Torso pose?
  local u0 = util.se2_interpolate(.5, uLeft_next, uRight_next)
  -- NOTE: supportX and supportY are globals
  local uLeftSupport  = util.pose_global({supportX,  supportY, 0}, uLeft_next )
  local uRightSupport = util.pose_global({supportX, -supportY, 0}, uRight_next)
  if supportLeg==0 then --Left support
    local uTorso_next = util.se2_interpolate(shiftFactor, uLeftSupport, uRightSupport)
    return uTorso_next
  else
    local uTorso_next = util.se2_interpolate(shiftFactor, uRightSupport, uLeftSupport)
    return uTorso_next
  end
end


function moveleg.step_destination_left(vel, uLeft, uRight)
  local uLRFootOffset = vector.new({0,footY,0})

  local u0 = util.se2_interpolate(.5, uLeft, uRight)
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = util.pose_global(vel, u0)
  local u2 = util.pose_global(.5*vel, u1)
  local uLeftPredict = util.pose_global(uLRFootOffset, u2)
  local uLeftRight = util.pose_relative(uLeftPredict, uRight)
  -- Do not pidgeon toe, cross feet:
  --Check toe and heel overlap
  local toeOverlap  = -footSizeX[1] * uLeftRight[3]
  local heelOverlap = -footSizeX[2] * uLeftRight[3]
  local limitY = math.max(stanceLimitY[1],
  stanceLimitY2+math.max(toeOverlap,heelOverlap))

  --print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)
  uLeftRight[1] = math.min(math.max(uLeftRight[1], stanceLimitX[1]), stanceLimitX[2])
  uLeftRight[2] = math.min(math.max(uLeftRight[2], limitY),stanceLimitY[2])
  uLeftRight[3] = math.min(math.max(uLeftRight[3], stanceLimitA[1]), stanceLimitA[2])

  return util.pose_global(uLeftRight, uRight)
end

function moveleg.step_destination_right(vel, uLeft, uRight)
  local uLRFootOffset = vector.new({0,footY,0})

  local u0 = util.se2_interpolate(.5, uLeft, uRight)
  -- Determine nominal midpoint position 1.5 steps in future
  local u1 = util.pose_global(vel, u0)
  local u2 = util.pose_global(.5*vel, u1)
  local uRightPredict = util.pose_global(-1*uLRFootOffset, u2)
  local uRightLeft = util.pose_relative(uRightPredict, uLeft)
  -- Do not pidgeon toe, cross feet:

  --Check toe and heel overlap
  local toeOverlap  = footSizeX[1] * uRightLeft[3]
  local heelOverlap = footSizeX[2] * uRightLeft[3]
  local limitY = math.max(stanceLimitY[1], stanceLimitY2+math.max(toeOverlap,heelOverlap))

  --print("Toeoverlap Heeloverlap",toeOverlap,heelOverlap,limitY)

  uRightLeft[1] = math.min(math.max(uRightLeft[1], stanceLimitX[1]), stanceLimitX[2])
  uRightLeft[2] = math.min(math.max(uRightLeft[2], -stanceLimitY[2]), -limitY)
  uRightLeft[3] = math.min(math.max(uRightLeft[3], -stanceLimitA[2]), -stanceLimitA[1])

  return util.pose_global(uRightLeft, uLeft)
end

function moveleg.get_gyro_feedback( uLeft, uRight, uTorsoActual, supportLeg )
  local body_yaw
  if supportLeg == 0 then  -- Left support
    body_yaw = uLeft[3]  - uTorsoActual[3]
  else
    body_yaw = uRight[3] - uTorsoActual[3]
  end
  -- Ankle stabilization using gyro feedback
  --local imu_roll0, imu_pitch0, imu_yaw0 = unpack(Body.get_sensor_imu())
  --math.sin(imuPitch)*bodyHeight, -math.sin(imuRoll)*bodyHeight
  local gyro_roll0, gyro_pitch0, gyro_yaw0 = unpack(Body.get_sensor_gyro())
  -- Get effective gyro angle considering body yaw offset
  -- Rotate the Roll and pitch about the intended body yaw
  local gyro_roll  = gyro_roll0  * math.cos(body_yaw) - gyro_pitch0 * math.sin(body_yaw)
  local gyro_pitch = gyro_pitch0 * math.cos(body_yaw) - gyro_roll0  * math.sin(body_yaw)


  -- Give these parameters
  return {gyro_roll, gyro_pitch, gyro_yaw0}
end



function moveleg.get_leg_compensation(supportLeg, phSingle, gyro_rpy,
    angleShift)
  local gyro_pitch = gyro_rpy[2]
  local gyro_roll = gyro_rpy[1]

  -- Ankle feedback
  local ankleShiftX = util.procFunc(gyro_pitch*ankleImuParamX[2],ankleImuParamX[3],ankleImuParamX[4])
  local ankleShiftY = util.procFunc(gyro_roll*ankleImuParamY[2],ankleImuParamY[3],ankleImuParamY[4])

  -- Ankle shift is filtered... thus a global
  angleShift[1] = angleShift[1] + ankleImuParamX[1]*(ankleShiftX-angleShift[1])
  angleShift[2] = angleShift[2] + ankleImuParamY[1]*(ankleShiftY-angleShift[2])

  -- Knee feedback
  local kneeShiftX = util.procFunc(gyro_pitch*kneeImuParamX[2],kneeImuParamX[3],kneeImuParamX[4])
  angleShift[3] = angleShift[3] + kneeImuParamX[1]*(kneeShiftX-angleShift[3])
  
  -- Hip feedback
  local hipShiftY=util.procFunc(gyro_roll*hipImuParamY[2],hipImuParamY[3],hipImuParamY[4])
  angleShift[4] = angleShift[4]+hipImuParamY[1]*(hipShiftY-angleShift[4])
  
  local delta_legs = vector.zeros(Body.nJointLLeg+Body.nJointRLeg)
  -- Change compensation in the beginning of the phase (first 10%)
  -- Saturate compensation afterwards
  -- Change compensation at the beginning of the phase (first 10%)
  -- Same sort of trapezoid at double->single->double support shape
  local phComp = 10 * math.min( phSingle, .1, 1-phSingle )

  if supportLeg == 0 then
    -- Left support
    delta_legs[2] = angleShift[4] + hipRollCompensation*phComp
    delta_legs[4] = angleShift[3]
    delta_legs[5] = angleShift[1]
    delta_legs[6] = angleShift[2]*phComp    
  elseif supportLeg==1 then    
    -- Right support
    delta_legs[8]  = angleShift[4] - hipRollCompensation*phComp
    delta_legs[10] = angleShift[3]
    delta_legs[11] = angleShift[1]
    delta_legs[12] = angleShift[2]*phComp    
  elseif supportLeg==2 then 
    -- Double support
    delta_legs[4] = angleShift[3]
    delta_legs[5] = angleShift[1]
    
    delta_legs[10] = angleShift[3]
    delta_legs[11] = angleShift[1]
  else --Robotis style

  end

--  print('Ankle shift',vector.new(ankleShift)*Body.RAD_TO_DEG )

  return delta_legs, angleShift
end

function moveleg.set_leg_positions(uTorso,uLeft,uRight,zLeft,zRight,delta_legs)
  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  local pTorso = vector.new({
        uTorsoActual[1], uTorsoActual[2], Config.walk.bodyHeight,
        0,Config.walk.bodyTilt,uTorsoActual[3]})
  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})
  local qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso)
  qLegs = qLegs + delta_legs
  Body.set_lleg_command_position(qLegs)

  ------------------------------------------
  -- Update the status in shared memory
  local uFoot = util.se2_interpolate(.5, uLeft, uRight)
  mcm.set_status_odometry( uFoot )
  --util.pose_relative(uFoot, u0) for relative odometry to point u0
  local bodyOffset = util.pose_relative(uTorso, uFoot)
  mcm.set_status_bodyOffset( bodyOffset )
  ------------------------------------------
end

function moveleg.set_leg_transforms(pLLeg,pRLeg,pTorso,supportLeg,delta_legs)
  local qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso, supportLeg)
  qLegs = qLegs + delta_legs
  Body.set_lleg_command_position(qLegs)
end  

function moveleg.get_ph_single(ph,phase1,phase2)
  return math.min(1, math.max(0, (ph-phase1)/(phase2-phase1) ))
end

function moveleg.foot_trajectory_base(phSingle,uStart,uEnd,stepHeight)
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle)
  local xf = .5*(1-math.cos(math.pi*phSingleSkew))
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew))
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf
  return uFoot, zFoot
end

function moveleg.foot_trajectory_square(phSingle,uStart,uEnd,stepHeight)
  local phase1,phase2 = 0.2, 0.7 --TODO: automatic detect
  local xf,zf = 0,0

  if phSingle<phase1 then --Lifting phase
    ph1 = phSingle / phase1
    zf = ph1;
  elseif phSingle<phase2 then
    ph1 = (phSingle-phase1) / (phase2-phase1)
    xf,zf = ph1, 1
  else
    ph1 = (phSingle-phase2) / (1-phase2)    
    xf,zf = 1, 1-ph1
  end
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf
  return uFoot, zFoot
end









function moveleg.get_foot(ph,start_phase,finish_phase)
  -- Computes relative x, z motion of foot during single support phase
  -- phSingle = 0: x=0, z=0, phSingle = 1: x=1,z=0
  -- phSingle is 100% @ finish_phase, and 0% at start_phase
  -- It just ignores the double support phase so we know how long we've been in single support
  local phSingle = math.min( math.max(ph-start_phase, 0)/(finish_phase-start_phase), 1)
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle)
  local xf = .5*(1-math.cos(math.pi*phSingleSkew))
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew))
  -- xf and zf and percentages, it seems
  return xf, zf, phSingle
end

function moveleg.get_foot_square(ph,start_phase,finish_phase)
  --SJ: Square wave walk pattern
  local phSingle = math.min( math.max(ph-start_phase, 0)/(finish_phase-start_phase), 1)
  local phase1 = 0.2;
  local phase2 = 0.7;

  if phSingle<phase1 then --Lifting phase
    ph1 = phSingle / phase1
    xf = 0;
    zf = ph1;
  elseif phSingle<phase2 then
    ph1 = (phSingle-phase1) / (phase2-phase1)
    xf = ph1;
    zf = 1;
  else
    ph1 = (phSingle-phase2) / (1-phase2)    
    xf = 1;
    zf = (1-ph1)
  end
  return xf,zf,phSingle
end


return moveleg

