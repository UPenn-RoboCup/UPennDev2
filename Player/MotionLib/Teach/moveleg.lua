local moveleg = {}
local Body   = require'Body'
local K = require'K_ffi'
local T = require'Transform'
local util = require'util'
local vector = require'vector'
require'mcm'
require'hcm'

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

local slow_p_tolerance = {
  -- x, y, z
  1e-3, 1e-3, 1e-3,
  -- r, p, y
  1*DEG_TO_RAD, 1*DEG_TO_RAD, 1*DEG_TO_RAD
}
-- How far away to tell the P controller to go in one step
local dqLegLimit = Config.stance.dqLegLimit
local dpLimitStance = Config.stance.dpLimitStance
local K0 = require'K_ffi_old'
local function set_lower_body_slowly(pTorso, pLLeg, pRLeg, dt)
  local zGround = mcm.get_status_zGround()
  -- Deal with the leg bias
	-- TODO: Fix the bias issue
  local legBias = 0*mcm.get_leg_bias()
  local legBiasL = vector.slice(legBias, 1, 6)
  local legBiasR = vector.slice(legBias, 7, 12)
  -- Where are we actually?
  local qL = Body.get_lleg_command_position()
  local qR = Body.get_rleg_command_position()
  local qLLegActual = qL - legBiasL
  local qRLegActual = qR - legBiasR
  -- How far away from the torso are the legs currently?
  local dpLLeg = T.position6D(T.inv(K.forward_l_leg(qLLegActual)))
  local dpRLeg = T.position6D(T.inv(K.forward_r_leg(qRLegActual)))	
  local pTorsoL = pLLeg + dpLLeg
  local pTorsoR = pRLeg + dpRLeg
  local pTorsoActual = (pTorsoL + pTorsoR) / 2
  -- Which torso to approach
  local pTorso_approach, doneTorso = util.approachTol(pTorsoActual, pTorso, dpLimitStance, dt, slow_p_tolerance)
  -- Where should the legs go?
  local qLLegTarget, qRLegTarget = K.inverse_legs(T.transform6D(pLLeg), T.transform6D(pRLeg), T.transform6D(pTorso_approach))
	
	qLLegTarget = qLLegTarget + legBiasL
	qRLegTarget = qRLegTarget + legBiasR
  local qLLegMove, doneL = util.approachTol(qLLegActual, qLLegTarget, dqLegLimit, dt)
  local qRLegMove, doneR = util.approachTol(qRLegActual, qRLegTarget, dqLegLimit, dt)
  -- Set the legs
  Body.set_lleg_command_position(qLLegMove)
  Body.set_rleg_command_position(qRLegMove)
  --[[
  print('TORSO APPROACH', pTorso, pTorso_approach)
  print('QLEG', qLLegTarget, qLLegActual)
  print('DONE', doneTorso, doneL, doneR)
  --]]
  -- Update our current situation
  mcm.set_stance_bodyTilt(pTorsoActual[5])
  mcm.set_stance_bodyHeightTarget(pTorsoActual[3])
  mcm.set_stance_bodyHeight(pTorsoActual[3])
  hcm.set_motion_bodyHeightTarget(pTorsoActual[3])
  -- Return the status
  return doneL and doneR and doneTorso
end
moveleg.set_lower_body_slowly = set_lower_body_slowly

--[[
uTorso:
uLeft:
uRight:
zLeft:
zRight:
dq: Limit the amount of movement (unit: radians NOT radians per second)
]]
function moveleg.set_leg_positions_slowly(uTorso, uLeft, uRight, zLeft, zRight, dt)
  local uTorsoActual = util.pose_global({-torsoX, 0, 0},uTorso)
  local pTorso = vector.new{
    -- XYZ
    uTorsoActual[1],
    uTorsoActual[2],
    mcm.get_stance_bodyHeight(),
    -- RPY
    0,
    mcm.get_stance_bodyTilt(),
    uTorsoActual[3]
  }
  local pLLeg = vector.new{uLeft[1],uLeft[2],zLeft,0,0,uLeft[3]}
  local pRLeg = vector.new{uRight[1],uRight[2],zRight,0,0,uRight[3]}
  return set_lower_body_slowly(pTorso, pLLeg, pRLeg, dt)
end



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
