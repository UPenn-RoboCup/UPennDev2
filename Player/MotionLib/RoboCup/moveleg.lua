local moveleg={}
local Body   = require'Body'
local K      = Body.Kinematics
local T      = require'Transform'
local util   = require'util'
local vector = require'vector'

require'mcm'

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
local hipRollCompensation = Config.walk.hipRollCompensation or 0
local ankleRollCompensation = Config.walk.ankleRollCompensation or 0
local anklePitchCompensation = Config.walk.anklePitchCompensation or 0
local kneePitchCompensation = Config.walk.kneePitchCompensation or 0
local hipPitchCompensation = Config.walk.hipPitchCompensation or 0



function moveleg.get_gyro_feedback( uLeft, uRight, uTorsoActual, supportLeg )
  local body_yaw
  if supportLeg == 0 then  -- Left support
    body_yaw = uLeft[3]  - uTorsoActual[3]
  else
    body_yaw = uRight[3] - uTorsoActual[3]
  end
  -- Ankle stabilization  gyro feedback
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

  local delta_legs = vector.zeros(12)
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

local comp_factor = 1
if mcm.get_stance_singlesupport()==1 then
  comp_factor = 2
end

if supportLeg == 0 then
    -- Left support
  delta_legs[2] = angleShift[4] + hipRollCompensation*phComp*comp_factor
elseif supportLeg==1 then
    -- Right support
  delta_legs[8]  = angleShift[4] - hipRollCompensation*phComp*comp_factor
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
























function moveleg.set_leg_positions_ankletilt(uTorso,uLeft,uRight,zLeft,zRight,delta_legs)

  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  local pTorso = vector.new({
        uTorsoActual[1], uTorsoActual[2], mcm.get_stance_bodyHeight(),
        0,mcm.get_stance_bodyTilt(),uTorsoActual[3]})
  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})

  --LeftHeel LeftToe RightHeel RightToe
  local footLift = K.calculate_foot_tilt(pLLeg, pRLeg, pTorso)
  local aLeg={0,0}
  local aLegOld = mcm.get_status_aLeg()

  --check which leg is forward
  local uLeftTorso = util.pose_relative(uLeft,uTorso)
  local uRightTorso = util.pose_relative(uRight,uTorso)

  if uLeft[1]>uRight[1] then --left foot forward
    aLeg = {footLift[2],footLift[3]} --Lfoot toe lift, right foot heel lift
  elseif uLeft[1]<uRight[1] then  --right foot forward
    aLeg = {footLift[1],footLift[4]} --Lfoot heel lift, right foot toe lift
  else
    aLeg = {footLift[2],footLift[4]} --Lfoot toe lift, right foot toe lift
  end

  --When foot is lifted, slowly zero ankle angle

  --TODO


  local qLegs = K.inverse_legs_foot_tilt(pLLeg, pRLeg, pTorso,aLeg)
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

  mcm.set_status_aLeg(aLeg)
  mcm.set_status_zLeg({zLeft,zRight})
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




function moveleg.foot_trajectory_walkkick(phSingle,uStart,uEnd,stepHeight)



local breaksTX={0.300000,0.600000,0.700000,0.800000,0.900000,1.000000,}
local breaksTY={0.300000,0.500000,0.700000,0.800000,0.900000,1.000000,}
local coefsX={
  {-5.566198,3.898467,1.664751,0.000000,},
  {-5.566198,-1.111111,2.500958,0.700000,},
  {28.065134,-6.120690,0.331418,1.200000,},
  {-17.911877,2.298851,-0.050766,1.200000,},
  {-6.417625,-3.074713,-0.128352,1.200000,},
  {-6.417625,-5.000000,-0.935824,1.150000,},
}
local coefsY={
  {5.646481,-9.517185,5.346972,0.000000,},
  {5.646481,-4.435352,1.161211,0.900000,},
  {-8.878887,-1.047463,0.064648,1.000000,},
  {5.728314,-6.374795,-1.419804,0.900000,},
  {-1.145663,-4.656301,-2.522913,0.700000,},
  {-1.145663,-5.000000,-3.488543,0.400000,},
}


local breaksTX={0.300000,0.400000,0.600000,0.800000,0.900000,1.000000,}
local breaksTY={0.300000,0.500000,0.700000,0.800000,0.900000,1.000000,}
local coefsX={
  {8.359213,0.815217,-0.163561,0.000000,},
  {8.359213,8.338509,2.582557,0.250000,},
  {-54.257246,10.846273,4.501035,0.600000,},
  {31.573499,-21.708075,2.328675,1.500000,},
  {34.213251,-2.763975,-2.565735,1.350000,},
  {34.213251,7.500000,-2.092133,1.100000,},
}
local coefsY={
  {5.646481,-9.517185,5.346972,0.000000,},
  {5.646481,-4.435352,1.161211,0.900000,},
  {-8.878887,-1.047463,0.064648,1.000000,},
  {5.728314,-6.374795,-1.419804,0.900000,},
  {-1.145663,-4.656301,-2.522913,0.700000,},
  {-1.145663,-5.000000,-3.488543,0.400000,},
}







  local xf=eval_spline(breaksTX, coefsX,phSingle)  
  local zf=eval_spline(breaksTY, coefsY,phSingle)  
  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  local zFoot = stepHeight * zf*1.5
  return uFoot, zFoot
end

--csapi([0 0.1 0.3 0.7 0.8 0.9 1],[0 -0.2 -1 2  2 1.4 1])
function moveleg.foot_trajectory_kick(phSingle,uStart,uEnd,stepHeight)
  local breaksTX={0.100000,0.300000,0.600000,0.700000,0.800000,0.900000,1.000000,}
  local breaksTY={0.100000,0.300000,0.500000,0.700000,0.800000,0.900000,1.000000,}
  local coefsX={
    {141.283466,-63.180053,2.905171,0.000000,},
    {141.283466,-20.795013,-5.492336,-0.200000,},
    {-137.068828,63.975066,3.143675,-1.000000,},
    {241.855712,-59.386879,4.520131,2.000000,},
    {-221.540977,13.169834,-0.101574,2.100000,},
    {244.308195,-53.292459,-4.113836,2.000000,},
    {244.308195,20.000000,-7.443082,1.300000,},
  }
  local coefsY={
    {36.771326,-33.041864,9.936473,0.000000,},
    {36.771326,-22.010466,4.431240,0.700000,},
    {-1.251969,0.052330,0.039613,1.000000,},
    {-6.763448,-0.698852,-0.089692,1.000000,},
    {-34.346163,-4.756921,-1.180846,0.900000,},
    {66.869233,-15.060770,-3.162615,0.700000,},
    {66.869233,5.000000,-4.168692,0.300000,},
  }


  --More swing back

local breaksTX={0.100000,0.400000,0.670000,0.720000,0.800000,0.900000,1.000000,}
local breaksTY={0.100000,0.300000,0.500000,0.700000,0.800000,0.900000,1.000000,}
local coefsX={
  {103.699049,-45.182858,-3.518705,0.000000,},
  {103.699049,-14.073143,-9.444305,-0.700000,},
  {-229.010397,79.256001,10.110553,-2.000000,},
  {995.160754,-106.242421,2.824219,2.000000,},
  {-485.346581,43.031692,-0.336317,2.000000,},
  {311.504957,-73.451487,-2.769901,2.000000,},
  {311.504957,20.000000,-8.115050,1.300000,},
}
local coefsY={
  {-32.433041,1.306550,7.193675,0.000000,},
  {-32.433041,-8.423363,6.481994,0.700000,},
  {108.898830,-27.883187,-0.779316,1.400000,},
  {-165.662278,37.456111,1.135269,1.000000,},
  {295.588566,-61.941256,-3.761760,1.400000,},
  {-39.117713,26.735314,-7.282354,0.700000,},
  {-39.117713,15.000000,-3.108823,0.200000,},
}



  local xf=eval_spline(breaksTX, coefsX,phSingle)  
  local zf=eval_spline(breaksTY, coefsY,phSingle)  
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
  local zFoot,aFoot = 0,0
  local zHeight0, zHeight1= 0,0,0
  local special = false


  if walkParam then    
    zHeight0, zHeight1 = walkParam[1],walkParam[3]
    stepHeight = walkParam[2]
    --hack for the special step for block climbing
    if walkParam[1]==-999 then
      zHeight0 = 0
      special = true
    end


    local move1 = math.abs(zHeight0-stepHeight)
    local move2 = math.abs(zHeight1-stepHeight)
  
    if move1>move2*2.0 then --step up
      phase1,phase2 = 0.5,0.8
    elseif move1*2.0<move2 then --step down
      phase1,phase2 = 0.2,0.5
    end
  end

  if phSingle<phase1 then --Lifting phase
    ph1 = phSingle / phase1
    zf = ph1;
    zFoot = zHeight0 + (stepHeight-zHeight0) * zf
    if special then
      if ph1<0.4 then ph2=ph1/0.4
      elseif ph1<0.7 then ph2 = 1
      else ph2 = (1-ph1)/0.3 end
      aFoot = 20*math.pi/180*ph2      
    end
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
  return uFoot, zFoot, aFoot
end

function moveleg.foot_trajectory_square_stair_touchdown(phSingle,uStart,uEnd, stepHeight, walkParam, zOld,forceZ, touched)
  local phase1,phase2 = 0.2, 0.7 --TODO: automatic detect
  local xf,zf = 0,0
  local zFoot,aFoot = 0,0
  local zHeight0, zHeight1= 0,0,0


  if walkParam then    
    zHeight0, zHeight1 = walkParam[1],walkParam[3]
    stepHeight = walkParam[2]
    local move1 = math.abs(zHeight0-stepHeight)
    local move2 = math.abs(zHeight1-stepHeight)
  
    if move1>move2*2.0 then --step up
      phase1,phase2 = 0.5,0.8
    elseif move1*2.0<move2 then --step down
      phase1,phase2 = 0.2,0.5
    end
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
    if forceZ>50 or touched or phSingle==1 then
      print("TOUCHDOWN!",zOld,forceZ)
      xf=1
      zFoot = zOld
      local uFoot = util.se2_interpolate(xf, uStart,uEnd)
      return uFoot, zFoot, aFoot, true
    end

    ph1 = (phSingle-phase2) / (1-phase2)
    xf,zf = 1, 1-ph1
    zFoot = zHeight1 + (stepHeight-zHeight1) * zf
  end

  local uFoot = util.se2_interpolate(xf, uStart,uEnd)
  return uFoot, zFoot, aFoot, false
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



function moveleg.get_leg_compensation_new(supportLeg, ph, gyro_rpy,angleShift,supportRatio,dt)

--New compensation code to cancelout backlash on ALL leg joints
  dt= dt or 0.010

  --Now we limit the angular velocity of compensation angles 
  local DEG_TO_RAD = math.pi/180
  local dShift = {30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD,30*DEG_TO_RAD}

  local gyro_pitch = gyro_rpy[2]
  local gyro_roll = gyro_rpy[1]

  -- Ankle feedback
  local ankleShiftX = util.procFunc(gyro_pitch*ankleImuParamX[2],ankleImuParamX[3],ankleImuParamX[4])
  local ankleShiftY = util.procFunc(gyro_roll*ankleImuParamY[2],ankleImuParamY[3],ankleImuParamY[4])
  -- Knee feedback
  local kneeShiftX = util.procFunc(gyro_pitch*kneeImuParamX[2],kneeImuParamX[3],kneeImuParamX[4])
  -- Hip feedback
  local hipShiftY=util.procFunc(gyro_roll*hipImuParamY[2],hipImuParamY[3],hipImuParamY[4])

  local dShiftTarget = {}
  dShiftTarget[1]=ankleImuParamX[1]*(ankleShiftX-angleShift[1])
  dShiftTarget[2]=ankleImuParamY[1]*(ankleShiftY-angleShift[2])
  dShiftTarget[3]=kneeImuParamX[1]*(kneeShiftX-angleShift[3])
  dShiftTarget[4]=hipImuParamY[1]*(hipShiftY-angleShift[4])
  
-- Ankle shift is filtered... thus a global
  angleShift[1] = angleShift[1] + math.max(-dShift[1]*dt,math.min(dShift[1]*dt,dShiftTarget[1]))
  angleShift[2] = angleShift[2] + math.max(-dShift[2]*dt,math.min(dShift[2]*dt,dShiftTarget[2])) 
  angleShift[3] = angleShift[3] + math.max(-dShift[3]*dt,math.min(dShift[3]*dt,dShiftTarget[3])) 
  angleShift[4] = angleShift[4] + math.max(-dShift[4]*dt,math.min(dShift[4]*dt,dShiftTarget[4])) 


  local delta_legs = vector.zeros(12)

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


  if mcm.get_stance_singlesupport()==1 then
    phComp = phComp*2
  end



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




function moveleg.set_leg_positions(uTorso,uLeft,uRight,zLeft,zRight,delta_legs,aLeft,aRight)


  local zShift=mcm.get_walk_zShift()
  local aShiftX=mcm.get_walk_aShiftX()
  local aShiftY=mcm.get_walk_aShiftY()

  zLeft = zLeft + zShift[1]
  zRight = zRight + zShift[2]


  local uTorsoActual = util.pose_global(vector.new({-torsoX,0,0}),uTorso)
  local pTorso = vector.new({
        uTorsoActual[1], uTorsoActual[2], mcm.get_stance_bodyHeight(),
        0,mcm.get_stance_bodyTilt(),uTorsoActual[3]})
  local pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]})
  local pRLeg = vector.new({uRight[1],uRight[2],zRight,0,0,uRight[3]})
  

  if aLeft then
    pLLeg = vector.new({uLeft[1],uLeft[2],zLeft,0,aLeft,uLeft[3]})
    pRLeg = vector.new({uRight[1],uRight[2],zRight,0,aRight,uRight[3]})
  end

  local qLegs = K.inverse_legs(pLLeg, pRLeg, pTorso)
  local legBias = vector.new(mcm.get_leg_bias())

  qLegs = qLegs + delta_legs + legBias

  qLegs[5]=qLegs[5]+aShiftY[1]
  qLegs[11]=qLegs[11]+aShiftY[2]
  qLegs[6]=qLegs[6]+aShiftX[1]
  qLegs[12]=qLegs[12]+aShiftX[2]


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


--Robotis style simple feedback
function moveleg.get_leg_compensation_simple(supportLeg, phSingle, gyro_rpy,angleShift)

  local gyro_pitch = gyro_rpy[2]
  local gyro_roll = gyro_rpy[1]

  local enable_gyro = hcm.get_legdebug_enable_gyro()
  if enable_gyro>0 then gyro_pitch,gyro_roll = 0,0 end

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

  local delta_legs = vector.zeros(12)
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

  return delta_legs, angleShift
end


function moveleg.ft_compensate(t_diff)

  local enable_balance = hcm.get_legdebug_enable_balance()
  local ft,imu = moveleg.get_ft()

  if enable_balance[1]+enable_balance[2]>0 then
    print()

    print(string.format("%d%d Fz: %d %d  T_p: %d %d T_r: %d %d", 
      enable_balance[1],enable_balance[2],
      ft.lf_z,ft.rf_z,ft.lt_y,ft.rt_y, ft.lt_x,ft.rt_x))
    print(string.format("angle: %.1f p %.1f",imu.roll_err*180/math.pi, imu.pitch_err*180/math.pi))
  end


--[[
  local t = Body.get_time()
  local t_last = mcm.get_walk_t_last()
  local t_diff = t-t_last
  mcm.set_walk_t_last(t)
--]]

  moveleg.process_ft_height(ft,imu,t_diff) -- height adaptation
  moveleg.process_ft_roll(ft,t_diff) -- roll adaptation
  moveleg.process_ft_pitch(ft,t_diff) -- pitch adaptation

end


function moveleg.get_ft()
  local y_angle_zero = 3*math.pi/180

  local l_ft, r_ft = Body.get_lfoot(), Body.get_rfoot()  
  local ft= {
    lf_z=l_ft[3],
    rf_z=r_ft[3],
    lt_x=-l_ft[4],
    rt_x=-r_ft[4],
    lt_y=l_ft[5],
    rt_Y=r_ft[5]
  }
  if IS_WEBOTS then
    ft.lt_y, ft.rt_y = -l_ft[4],-r_ft[5]      
    ft.lt_x,ft.rt_x = -l_ft[5],-r_ft[4] 
  end
  local rpy = Body.get_rpy()
  local gyro, gyro_t = Body.get_gyro()
  local imu={
    roll_err = rpy[1],
    pitch_err = rpy[2]-y_angle_zero,
    v_roll = gyro[1],
    v_pitch = gyro[2]
  }
  return ft,imu
end



function moveleg.process_ft_height(ft,imu,t_diff)
  --------------------------------------------------------------------------------------------------------
  -- Foot height differential adaptation

  local zf_touchdown = 50
  local z_shift_max = 0.05 --max 5cm difference
  local z_vel_max_diff = 0.4 --max 40cm per sec
  local z_vel_max_balance = 0.05 --max 5cm per sec
  local k_const_z_diff = 0.5 / 100  -- 50cm/s for 100 N difference
  local z_shift_diff_db = 50 --50N deadband
  local k_balancing = 0.4 


  local enable_balance = hcm.get_legdebug_enable_balance()

  local uLeft = mcm.get_status_uLeft()
  local uRight = mcm.get_status_uRight()
  local uTorso = mcm.get_status_uTorso()  

  local uLeftTorso = util.pose_relative(uLeft,uTorso)
  local uRightTorso = util.pose_relative(uRight,uTorso)

  local zvShift={0,0}
  local balancing_type=0


  local enable_adapt = false


  if ((ft.lf_z<zf_touchdown and ft.rf_z>zf_touchdown) or (ft.lf_z>zf_touchdown and ft.rf_z<zf_touchdown) )
    and math.abs(imu.roll_err)<2*math.pi/180
    and enable_adapt

    then 
    

    local zvShiftTarget = util.procFunc( (ft.lf_z-ft.rf_z)*k_const_z_diff , z_shift_diff_db*k_const_z_diff, z_vel_max_diff)
    if enable_balance[1]>0 then zvShift[1] = zvShiftTarget end
    if enable_balance[2]>0 then zvShift[2] = -zvShiftTarget end      
  else
    balancing_type=1
    --both feet on the ground. use IMU to keep torso orientation up

    local LR_pitch_err = -(uLeftTorso[1]-uRightTorso[1])*math.tan(imu.pitch_err)
    local LR_roll_err =  (uLeftTorso[2]-uRightTorso[2])*math.tan(imu.roll_err)
    local zvShiftTarget = util.procFunc( (LR_pitch_err + LR_roll_err) * k_balancing, 0, z_vel_max_balance )

    if enable_balance[1]>0 then zvShift[1] = zvShiftTarget end
    if enable_balance[2]>0 then zvShift[2] = -zvShiftTarget end  
  end

  local zShift = mcm.get_walk_zShift()
  zShift[1] = util.procFunc( zShift[1]+zvShift[1]*t_diff , 0, z_shift_max)
  zShift[2] = util.procFunc( zShift[2]+zvShift[2]*t_diff , 0, z_shift_max)
  mcm.set_walk_zShift(zShift)
  mcm.set_walk_zvShift(zvShift)

  if enable_balance[1]+enable_balance[2]>0 then
    if balancing_type>0 then print"DS balancing" end
    print(string.format("dZ : %.1f Zshift: %.2f %.2f cm",zvShift[1],zShift[1]*100,zShift[2]*100))
  end
  --------------------------------------------------------------------------------------------------------
end

function moveleg.process_ft_roll(ft,t_diff)

  local k_const_tx =   20 * math.pi/180 /5  --Y angular spring constant: 20 deg/s  / 5 Nm
  local r_const_tx =   0 --zero damping for now  
  local ax_shift_db =  0.3 -- 0.3Nm deadband
  local ax_vel_max = 30*math.pi/180 
  local ax_shift_max = 30*math.pi/180

  ----------------------------------------------------------------------------------------
  -- Ankle roll adaptation 

  local aShiftX=mcm.get_walk_aShiftX()
  local avShiftX=mcm.get_walk_avShiftX()

  local enable_balance = hcm.get_legdebug_enable_balance()

  avShiftX[1] = util.procFunc( ft.lt_x*k_const_tx + avShiftX[1]*r_const_tx    
      ,k_const_tx*ax_shift_db, ax_vel_max)
  avShiftX[2] = util.procFunc( ft.rt_x*k_const_tx + avShiftX[2]*r_const_tx    
      ,k_const_tx*ax_shift_db, ax_vel_max)

  if enable_balance[1]>0 then
    aShiftX[1] = aShiftX[1]+avShiftX[1]*t_diff
    aShiftX[1] = math.min(ax_shift_max,math.max(-ax_shift_max,aShiftX[1]))
  end

  if enable_balance[2]>0 then
    aShiftX[2] = aShiftX[2]+avShiftX[2]*t_diff
    aShiftX[2] = math.min(ax_shift_max,math.max(-ax_shift_max,aShiftX[2]))
  end

  mcm.set_walk_aShiftX(aShiftX)
  mcm.set_walk_avShiftX(avShiftX)

  if enable_balance[1]+enable_balance[2]>0 then
  print(string.format("dRoll: %.1f %.1f Roll: %.1f %.1f",
    avShiftX[1]*180/math.pi,avShiftX[2]*180/math.pi,
    aShiftX[1]*180/math.pi,aShiftX[2]*180/math.pi))   
  end

  ----------------------------------------------------------------------------------------

end

function moveleg.process_ft_pitch(ft,t_diff)

  local k_const_ty =  4 *   math.pi/180   --Y angular spring constant: 20 deg/s  / 5 Nm
  local r_const_ty =   0 --zero damping for now
  local ay_shift_max = 30*math.pi/180
  local ay_shift_db = 1*math.pi/180
  local ay_vel_max = 30*math.pi/180 

  ----------------------------------------------------------------------------------------
  -- Ankle pitch adaptation 
  local aShiftY=mcm.get_walk_aShiftY()
  local avShiftY=mcm.get_walk_avShiftY()

  local enable_balance = hcm.get_legdebug_enable_balance()
    
  avShiftY[1] = util.procFunc(  ft.lt_y*k_const_ty + avShiftY[1]*r_const_ty    
      ,k_const_ty*ay_shift_db, ay_vel_max)
  avShiftY[2] = util.procFunc(  ft.rt_y*k_const_ty + avShiftY[2]*r_const_ty    
      ,k_const_ty*ay_shift_db, ay_vel_max)

  if enable_balance[1]>0 then
    aShiftY[1] = aShiftY[1]+avShiftY[1]*t_diff
    aShiftY[1] = math.min(ay_shift_max,math.max(-ay_shift_max,aShiftY[1]))
  end

  if enable_balance[2]>0 then
    aShiftY[2] = aShiftY[2]+avShiftY[2]*t_diff
    aShiftY[2] = math.min(ay_shift_max,math.max(-ay_shift_max,aShiftY[2]))
  end

  mcm.set_walk_aShiftY(aShiftY)
  mcm.set_walk_avShiftY(avShiftY)

  if enable_balance[1]+enable_balance[2]>0 then
    print(string.format("dPitch: %.1f %.1f Pitch: %.1f %.1f",
      avShiftY[1]*180/math.pi,avShiftY[2]*180/math.pi,
      aShiftY[1]*180/math.pi,aShiftY[2]*180/math.pi))   
  end
  ----------------------------------------------------------------------------------------

end






return moveleg
