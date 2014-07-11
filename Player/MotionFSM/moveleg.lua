local moveleg={}
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'

-- SJ: Shared library for 2D leg trajectory generation
-- So that we can reuse them for different controllers

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
local ankleRollCompensation = Config.walk.ankleRollCompensation
local anklePitchCompensation = Config.walk.anklePitchCompensation
local kneePitchCompensation = Config.walk.kneePitchCompensation
local hipPitchCompensation = Config.walk.hipPitchCompensation



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
  local gyro, gyro_t = Body.get_gyro()
  local gyro_roll0, gyro_pitch0, gyro_yaw0 = unpack(gyro)
  -- Get effective gyro angle considering body yaw offset
  -- Rotate the Roll and pitch about the intended body yaw
  local gyro_roll  = gyro_roll0  * math.cos(body_yaw) - gyro_pitch0 * math.sin(body_yaw)
  local gyro_pitch = gyro_pitch0 * math.cos(body_yaw) - gyro_roll0  * math.sin(body_yaw)


  -- Give these parameters
  return {gyro_roll, gyro_pitch, gyro_yaw0}
end



function moveleg.get_leg_compensation(supportLeg, ph, gyro_rpy,angleShift)
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

  --SJ: now we apply the compensation during DS too
  local phComp1 = 0.1
  local phComp2 = 0.9
  local phSingleComp = math.min( math.max(ph-phComp1, 0)/(phComp2-phComp1), 1)

  local phComp = 10 * math.min( phSingleComp, .1, 1-phSingleComp)
--[[
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
    delta_legs[2] = angleShift[4]
    delta_legs[4] = angleShift[3]
    delta_legs[5] = angleShift[1]
    delta_legs[6] = angleShift[2]

    delta_legs[8]  = angleShift[4]
    delta_legs[10] = angleShift[3]
    delta_legs[11] = angleShift[1]
    delta_legs[12] = angleShift[2]
  end
--]]

if supportLeg == 0 then
    -- Left support
  delta_legs[2] = angleShift[4] + hipRollCompensation*phComp
elseif supportLeg==1 then
    -- Right support
  delta_legs[8]  = angleShift[4] - hipRollCompensation*phComp
else
  delta_legs[2] = angleShift[4]
  delta_legs[8]  = angleShift[4]
end

delta_legs[4] = angleShift[3]
delta_legs[5] = angleShift[1]
delta_legs[6] = angleShift[2]

delta_legs[10] = angleShift[3]
delta_legs[11] = angleShift[1]
delta_legs[12] = angleShift[2]


--  print('Ankle shift',angleShift[1]*Body.RAD_TO_DEG )

  return delta_legs, angleShift
end



--Robotis style simple feedback
function moveleg.get_leg_compensation_simple(supportLeg, phSingle, gyro_rpy,angleShift)
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

  delta_legs[4] = angleShift[3]
  delta_legs[5] = angleShift[1]

  delta_legs[10] = angleShift[3]
  delta_legs[11] = angleShift[1]

 delta_legs[2] = angleShift[4]
 delta_legs[6] = angleShift[2]
  delta_legs[8]  = angleShift[4]
  delta_legs[12] = angleShift[2]
  --[[
  if supportLeg == 0 then -- Left support
    delta_legs[2] = angleShift[4]
    delta_legs[2] = delta_legs[2] + hipRollCompensation*phComp
    delta_legs[6] = angleShift[2]
  elseif supportLeg==1 then    -- Right support
    delta_legs[8]  = angleShift[4]
    delta_legs[8]  = delta_legs[8] - hipRollCompensation*phComp
    delta_legs[12] = angleShift[2]
  elseif supportLeg==2 then

  end
  --]]

--  print('Ankle shift',angleShift[1]*Body.RAD_TO_DEG )

  return delta_legs, angleShift
end



function moveleg.set_leg_positions(uTorso,uLeft,uRight,zLeft,zRight,delta_legs)
  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  local pTorso = vector.new({
        uTorsoActual[1], uTorsoActual[2], mcm.get_stance_bodyHeight(),
        0,mcm.get_stance_bodyTilt(),uTorsoActual[3]})
  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})
  local qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso)
  local legBias = vector.new(mcm.get_leg_bias())

  qLegs = qLegs + delta_legs + legBias

  Body.set_lleg_command_position(vector.slice(qLegs,1,6))
  Body.set_rleg_command_position(vector.slice(qLegs,7,12))

  ------------------------------------------
  -- Update the status in shared memory
  local uFoot = util.se2_interpolate(.5, uLeft, uRight)
  mcm.set_status_odometry( uFoot )
  --util.pose_relative(uFoot, u0) for relative odometry to point u0
  local bodyOffset = util.pose_relative(uTorso, uFoot)
  mcm.set_status_bodyOffset( bodyOffset )
  ------------------------------------------
end

function moveleg.set_leg_positions_kneel(dt)
  local uTorso = mcm.get_status_uTorso()
  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()

  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)

  local bodyHeightVel = 0.01

  local bodyHeight0 = mcm.get_stance_bodyHeight()
  local bodyHeight1 = mcm.get_stance_bodyHeight() + dt*bodyHeightVel
  local bodyHeight2 = mcm.get_stance_bodyHeight() - dt*bodyHeightVel

  local pTorso0 = vector.new({
        uTorsoActual[1], uTorsoActual[2], bodyHeight0,
        0,mcm.get_stance_bodyTilt(),uTorsoActual[3]})

  local pTorso1 = vector.new({
        uTorsoActual[1], uTorsoActual[2], bodyHeight1,
        0,mcm.get_stance_bodyTilt(),uTorsoActual[3]})

  local pTorso2 = vector.new({
        uTorsoActual[1], uTorsoActual[2], bodyHeight2,
        0,mcm.get_stance_bodyTilt(),uTorsoActual[3]})


  local pLLeg = vector.new({uLeft[1],uLeft[2],0,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],0,0,0,uRight[3]})

  local qLegs0 = K.inverse_legs(pLLeg, pRLeg, pTorso0)
  local qLegs1 = K.inverse_legs(pLLeg, pRLeg, pTorso1)
  local qLegs2 = K.inverse_legs(pLLeg, pRLeg, pTorso2)

  local kneePadding = 0.08

  local kneeHeight0 = K.calculate_knee_height(vector.slice(qLegs0,1,6)) - kneePadding
  local kneeHeight1 = K.calculate_knee_height(vector.slice(qLegs1,1,6)) - kneePadding
  local kneeHeight2 = K.calculate_knee_height(vector.slice(qLegs2,1,6)) - kneePadding

  if kneeHeight0>0 then
    if kneeHeight2>0 and kneeHeight2<kneeHeight0 then
      mcm.set_stance_bodyHeight(bodyHeight2)
      Body.set_lleg_command_position(qLegs2)
    else
      Body.set_lleg_command_position(qLegs0)
    end
  else
    if kneeHeight1<0 and kneeHeight1>kneeHeight0 then
      mcm.set_stance_bodyHeight(bodyHeight1)
      Body.set_lleg_command_position(qLegs1)
    else
      Body.set_lleg_command_position(qLegs0)
    end
  end

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


local function eval_spline(breaks,coefs,ph)
  local x_offset, xf = 0,0
  for i=1,#breaks do
    if ph<=breaks[i] then
      local x=ph - x_offset
      xf = coefs[i][1]*x^3 + coefs[i][2]*x^2 + coefs[i][3]*x + coefs[i][4]
      break;
    end
    x_offset = breaks[i]    
  end
  return xf
end



--a=csapi([0 0.1 0.3 0.6 0.8 0.9 1],[0 -0.1 -0.3 1 1.5 1.2 1])
function moveleg.foot_trajectory_walkkick(phSingle,uStart,uEnd,stepHeight)
  local breaksX={0.1 ,0.3 ,0.6,0.8,0.9,1}
  local coefsX = {
    {36.9313,-14.7725,0.1079,0},
    {36.9313,-3.6931,-1.7386,-0.10},
    {-26.9139,18.4637,1.2159,-0.3000},
    {-34.4293,-5.7568,5.0285,1},
    {104.7146,-26.4144,-1.4057,1.5},
    {104.7146,5,-3.5471,1.2},
  }
  local xf=eval_spline(breaksX, coefsX,phSingle)
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle)
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew))
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf*1.5

  return uFoot, zFoot
end

--csapi([0 0.1 0.3 0.7 0.8 0.9 1],[0 -0.2 -1 2  2 1.4 1])
function moveleg.foot_trajectory_kick(phSingle,uStart,uEnd,stepHeight)
  local breaksX={0.1,0.3,0.7,0.8,0.9,1}
  local coefsX = {
      {100.4549 , -46.8486  ,  1.6803  ,       0},
      {100.4549 , -16.7122 ,  -4.6758  , -0.2000},
      {-66.3641 ,  43.5608  ,  0.6940  , -1.0000},
      {-8.0158 , -36.0762  ,  3.6878   , 2.0000},
      { 161.6032 , -38.4809  , -3.7679  ,  2.0000},
      {161.6032 ,  10.0000 ,  -6.6160   , 1.4000},
    }
  local xf=eval_spline(breaksX, coefsX,phSingle)  
  local phSingleSkew = phSingle^0.8 - 0.17*phSingle*(1-phSingle)
  local zf = .5*(1-math.cos(2*math.pi*phSingleSkew))
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf*2.5
  return uFoot, zFoot
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


function moveleg.foot_trajectory_square_stair(phSingle,uStart,uEnd, stepHeight, walkParam)
  local phase1,phase2 = 0.2, 0.7 --TODO: automatic detect
  local xf,zf = 0,0
  local zFoot
  local zHeight0, zHeight1= 0,0,0

  if walkParam then
    zHeight0, zHeight1 = walkParam[1],walkParam[3]
    stepHeight = walkParam[2]
  end

  if phSingle<phase1 then --Lifting phase
    ph1 = phSingle / phase1
    zf = ph1;
    zFoot = zHeight0 + (stepHeight-zHeight0) * zf
  elseif phSingle<phase2 then
    ph1 = (phSingle-phase1) / (phase2-phase1)
    xf,zf = ph1, 1
    zFoot = stepHeight * zf
  else
    ph1 = (phSingle-phase2) / (1-phase2)
    xf,zf = 1, 1-ph1
    zFoot = zHeight1 + (stepHeight-zHeight1) * zf
  end

  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  return uFoot, zFoot
end



function moveleg.foot_trajectory_square_stair_2(phSingle,uStart,uEnd, stepHeight, walkParam)
  local phase1,phase2 = 0.2, 0.7 --TODO: automatic detect
  local xf,zf = 0,0
  local zFoot
  local zHeight0, zHeight1= 0,0,0


  if walkParam then
    zHeight0, zHeight1 = walkParam[1],walkParam[3]
    stepHeight = walkParam[2]

    local d0 = math.abs(walkParam[1]-walkParam[2])
    local uFootMovement = util.pose_relative(uEnd,uStart)
    local d1 = math.sqrt(uFootMovement[1]*uFootMovement[1] + uFootMovement[2]*uFootMovement[2])
    local d2 = math.abs(walkParam[3]-walkParam[2])

    --New linear speed foot movement

    local dTotal = d0+d1+d2; --Total foot movement distance
    local phase0 = d0 / dTotal;    --Lifting phase
    local phase1 = d0+d1 / dTotal; --Movement phase
    local phase2 = 1               --Final stepdown phase

--print("Phases:",phase0,phase1,phase2,phase3)

    --Slow lifting/landing
    --TODO
--[[
    local liftFactor = 1.5;
    local landFactor = 2.0;
    local phSingle2
    if phSingle<phase0 then
      phSingle2 = phSingle / liftFactor
    elseif phSingle<phase1 then
      phSingle2 = phSingle / liftFactor
    else
      phSingle2 = 1- (1-phSingle)/landFactor
    end
--]]
    local phSingle2 = phSingle

    if phSingle2<phase0 then --Lifting phase
      local ph1 = phSingle2/phase0
      xf,zf = 0, ph1;
      zFoot = zHeight0 + (stepHeight-zHeight0) * zf
    elseif phSingle2<phase1 then --Movement phase
      local ph1 = (phSingle2-phase0)/(phase1-phase0)
      xf,zf = ph1, 1
      zFoot = stepHeight
    else --landing phase
      local ph1 = (phSingle2-phase1)/(phase2-phase1)
      xf,zf = 1, 1-ph1
      zFoot = zHeight1 + (stepHeight-zHeight1) * zf
    end

  else
    if phSingle<phase1 then --Lifting phase
      ph1 = phSingle / phase1
      zf = ph1;
      zFoot = zHeight0 + (stepHeight-zHeight0) * zf
    elseif phSingle<phase2 then
      ph1 = (phSingle-phase1) / (phase2-phase1)
      xf,zf = ph1, 1
      zFoot = stepHeight * zf
    else
      ph1 = (phSingle-phase2) / (1-phase2)
      xf,zf = 1, 1-ph1
      zFoot = zHeight1 + (stepHeight-zHeight1) * zf
    end
  end
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  return uFoot, zFoot
end




function moveleg.foot_trajectory_square_touchdown(phSingle,uStart,uEnd,stepHeight, touched)
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



function moveleg.get_leg_compensation_new(supportLeg, ph, gyro_rpy,angleShift,supportRatio)

--New compensation code to cancelout backlash on ALL leg joints


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

  --How much do we need to apply the compensation?
  local supportRatioRight = supportRatio;
  local supportRatioLeft = 1-supportRatio;
--  supportRatioLeft = math.max(0,supportRatioLeft*4-3);
--  supportRatioRight = math.max(0,supportRatioRight*4-3);

  supportRatioLeft = math.max(0,supportRatioLeft*2-1);
  supportRatioRight = math.max(0,supportRatioRight*2-1);



--print("SR:",supportRatio,supportRatioLeft,supportRatioRight)
  --SJ: now we apply the compensation during DS too
  local phComp1 = Config.walk.phComp[1]
  local phComp2 = Config.walk.phComp[2]
  local phCompSlope = Config.walk.phCompSlope

  local phSingleComp = math.min( math.max(ph-phComp1, 0)/(phComp2-phComp1), 1)
  local phComp = math.min( phSingleComp/phCompSlope, 1,
                          (1-phSingleComp)/phCompSlope)
  supportRatioLeft, supportRatioRight = 0,0
  if supportLeg == 0 then -- Left support
    supportRatioLeft = phComp;
  elseif supportLeg==1 then
    supportRatioRight = phComp;
  end

    delta_legs[2] = angleShift[4] + hipRollCompensation*supportRatioLeft
    delta_legs[3] = - hipPitchCompensation*supportRatioLeft
    delta_legs[4] = angleShift[3] - kneePitchCompensation*supportRatioLeft
    delta_legs[5] = angleShift[1] - anklePitchCompensation*supportRatioLeft
    delta_legs[6] = angleShift[2] + ankleRollCompensation*supportRatioLeft

    delta_legs[8]  = angleShift[4] - hipRollCompensation*supportRatioRight
    delta_legs[9] = -hipPitchCompensation*supportRatioRight
    delta_legs[10] = angleShift[3] - kneePitchCompensation*supportRatioRight
    delta_legs[11] = angleShift[1] - anklePitchCompensation*supportRatioRight
    delta_legs[12] = angleShift[2] - ankleRollCompensation

  return delta_legs, angleShift
end


return moveleg
