--------------------------------
-- Lua Kinematics for THOR-OP
-- (c) 2014 Stephen McGill, Seung-Joon Yi
--------------------------------
-- TODO: Add ability to change the hand offsets, which affects preArm, postArm transforms
-- TODO: No automatic toe lift when limit reached, yet
local K = {}
-- Cache needed functions
local Tnew = require'Transform'.new
local Tinv = require'Transform'.inv
local TrotX = require'Transform'.rotX
local TrotY = require'Transform'.rotY
local TrotZ = require'Transform'.rotZ
local Ttrans = require'Transform'.trans
local vnew = require'vector'.new
local sin, cos = require'math'.sin, require'math'.cos
local asin, acos = require'math'.asin, require'math'.acos
local sqrt, atan2, atan = require'math'.sqrt, require'math'.atan2, require'math'.atan
local PI = require'math'.pi
local TWO_PI = 2 * PI
local min, max = require'math'.min, require'math'.max
-- Arm constants
local shoulderOffsetX = 0;
local shoulderOffsetY = 0.234;
local shoulderOffsetZ = 0.165;
local upperArmLength = 0.246;
local elbowOffsetX = 0.030;
--local lowerArmLength = 0.186; -- Default 7DOF arm
local lowerArmLength = 0.250; -- LONGARM model
-- Gripper of no appendage - just the plate
local handOffsetX = 0.125;--0.245;
local handOffsetY = 0;--0.035;
local handOffsetZ = 0;
-- Leg constants
local hipOffsetX = 0
local hipOffsetY = 0.072
local hipOffsetZ = 0.282;
local thighLength = 0.30
local tibiaLength = 0.30
local kneeOffsetX = 0.03
local footHeight = 0.118 -- Webots value
local footToeX = 0.130 -- from ankle to toe
local footHeelX = 0.130 -- from ankle to heel
local dThigh = sqrt(thighLength*thighLength+kneeOffsetX*kneeOffsetX)
local aThigh = atan(kneeOffsetX/thighLength)
local dTibia = sqrt(tibiaLength*tibiaLength+kneeOffsetX*kneeOffsetX)
local aTibia = atan(kneeOffsetX/tibiaLength)

-- Sanitize to avoid trouble with wrist yaw
local fabs = math.abs
local function mod_angle(a)
  -- Reduce angle to [-pi, pi)
  local b = a % TWO_PI
  return b>=PI and (b - TWO_PI) or b
end
function K.sanitize(iqArm, cur_qArm)
	local diff, mod_diff
	for i, v in ipairs(cur_qArm) do
		diff = iqArm[i] - v
		mod_diff = mod_angle(diff)
    iqArm[i] = (fabs(diff) > fabs(mod_diff)) and v + mod_diff or iqArm[i]
	end
end

local function fk_arm(q)
	local c1, s1 = cos(q[1]), sin(q[1])
	local c2, s2 = cos(q[2]), sin(q[2])
	local c3, s3 = cos(q[3]), sin(q[3])
	local c4, s4 = cos(q[4]), sin(q[4])
	local c5, s5 = cos(q[5]), sin(q[5])
	local c6, s6 = cos(q[6]), sin(q[6])
	local c7, s7 = cos(q[7]), sin(q[7])
	return Tnew{
		{(((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*s5 + (s1*s3 - s2*c1*c3)*c5)*s6 + (-(s1*c3 + s2*s3*c1)*s4 + c1*c2*c4)*c6, ((((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*s5 + (s1*s3 - s2*c1*c3)*c5)*c6 - (-(s1*c3 + s2*s3*c1)*s4 + c1*c2*c4)*s6)*c7 + (((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*c5 - (s1*s3 - s2*c1*c3)*s5)*s7, -((((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*s5 + (s1*s3 - s2*c1*c3)*c5)*c6 - (-(s1*c3 + s2*s3*c1)*s4 + c1*c2*c4)*s6)*s7 + (((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*c5 - (s1*s3 - s2*c1*c3)*s5)*c7, -elbowOffsetX*((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2) + elbowOffsetX*(s1*c3 + s2*s3*c1) + lowerArmLength*(-(s1*c3 + s2*s3*c1)*s4 + c1*c2*c4) + upperArmLength*c1*c2},
		{((s2*s4 - s3*c2*c4)*s5 + c2*c3*c5)*s6 + (s2*c4 + s3*s4*c2)*c6, (((s2*s4 - s3*c2*c4)*s5 + c2*c3*c5)*c6 - (s2*c4 + s3*s4*c2)*s6)*c7 + ((s2*s4 - s3*c2*c4)*c5 - s5*c2*c3)*s7, -(((s2*s4 - s3*c2*c4)*s5 + c2*c3*c5)*c6 - (s2*c4 + s3*s4*c2)*s6)*s7 + ((s2*s4 - s3*c2*c4)*c5 - s5*c2*c3)*c7, -elbowOffsetX*(s2*s4 - s3*c2*c4) - elbowOffsetX*s3*c2 + lowerArmLength*(s2*c4 + s3*s4*c2) + upperArmLength*s2},
		{(((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*s5 + (s1*s2*c3 + s3*c1)*c5)*s6 + (-(-s1*s2*s3 + c1*c3)*s4 - s1*c2*c4)*c6, ((((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*s5 + (s1*s2*c3 + s3*c1)*c5)*c6 - (-(-s1*s2*s3 + c1*c3)*s4 - s1*c2*c4)*s6)*c7 + (((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*c5 - (s1*s2*c3 + s3*c1)*s5)*s7, -((((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*s5 + (s1*s2*c3 + s3*c1)*c5)*c6 - (-(-s1*s2*s3 + c1*c3)*s4 - s1*c2*c4)*s6)*s7 + (((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*c5 - (s1*s2*c3 + s3*c1)*s5)*c7, -elbowOffsetX*((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2) + elbowOffsetX*(-s1*s2*s3 + c1*c3) + lowerArmLength*(-(-s1*s2*s3 + c1*c3)*s4 - s1*c2*c4) - upperArmLength*s1*c2},
		{0, 0, 0, 1}}
end
K.forward_arm = fk_arm

-- Precalculate some stuff
local trans_upper = Ttrans(upperArmLength, 0, elbowOffsetX)
local trans_lower = Ttrans(lowerArmLength, 0, -elbowOffsetX)
local dUpperArm = sqrt(upperArmLength^2 + elbowOffsetX^2)
local dLowerArm = sqrt(lowerArmLength^2 + elbowOffsetX^2)
local aUpperArm = atan(elbowOffsetX / upperArmLength)
local aLowerArm = atan(elbowOffsetX / lowerArmLength)
local aElbowMax = -1*(aUpperArm + aLowerArm)
-- Assume left arm
local function ik_arm(trArm, qOrg, shoulderYaw, FLIP_SHOULDER_ROLL)
	local xWrist1, xWrist2, xWrist3 = trArm[1][4], trArm[2][4], trArm[3][4]

  -- SJ: Robot can have TWO elbow pitch values (near elbowPitch==0)
  -- We are only using the smaller one (arm more bent)
  local dWrist = xWrist1^2 + xWrist2^2 + xWrist3^2
  local cElbow = (dWrist-dUpperArm^2 - dLowerArm^2)/(2*dUpperArm*dLowerArm)

	cElbow = max(-1, min(1, cElbow))

  local elbowPitch = -acos(cElbow) - aUpperArm - aLowerArm

  -- From shoulder yaw to wrist
  local m = TrotX(shoulderYaw) * trans_upper * TrotY(elbowPitch) * trans_lower
  local a = m[1][4]^2 + m[2][4]^2
  local b = -m[1][4] * xWrist2
  local c = xWrist2^2 - m[2][4]^2
  --
  local shoulderRoll
  -- NaN handling
	local det = b^2 - a*c
  if det < 0 or a==0 then
    shoulderRoll = 0
  else
		local sqrt_det = sqrt(det)
    local s21 = (-b + sqrt_det)/a
    local s22 = (-b - sqrt_det)/a
		s21 = max(-1, min(1, s21))
		s22 = max(-1, min(1, s22))
    local shoulderRoll1, shoulderRoll2 = asin(s21), asin(s22)
    local err1 = s21*m[1][4] + cos(shoulderRoll1)*m[2][4] - xWrist2
    local err2 = s22*m[1][4] + cos(shoulderRoll2)*m[2][4] - xWrist2
    shoulderRoll = err1^2 < err2^2 and shoulderRoll1 or shoulderRoll2
  end
	shoulderRoll = FLIP_SHOULDER_ROLL and (FLIP_SHOULDER_ROLL - shoulderRoll) or shoulderRoll

  local t1 = m[2][4] * sin(shoulderRoll) - m[1][4] * cos(shoulderRoll)

  local m23 = m[3][4]
  local c1 = (m23*xWrist3 - t1*xWrist1) / (m23^2 + t1^2)
  local s1 = (m23*xWrist1 + t1*xWrist3) / (m23^2 + t1^2)

  local shoulderPitch = atan2(s1, c1)

	local qArm = {shoulderPitch, shoulderRoll, shoulderYaw, elbowPitch}

	--
	-- Now find the wrist
	--
  -- Now we know shoulder pich, roll, yaw and elbow pitch
  -- Calc the final transform for the wrist based on rotation alone
	local rotWrist = TrotY(-elbowPitch) * TrotX(-shoulderYaw) * TrotZ(-shoulderRoll) * TrotY(-shoulderPitch) * trArm

  -- NOTE: singular point: just use current angles
	local wristYaw_a, wristYaw2_a, wristRoll_b, wristYaw_b, wristYaw2_b
	local wristRoll_a = acos(rotWrist[1][1]) -- 0 to pi
  if sin(wristRoll_a) < 1e-5 then
	--if (wristRoll_a) < 1e-5 then
    wristYaw_a = qOrg[5]
    wristRoll_a = qOrg[6]
    wristYaw2_a = qOrg[7]
    wristYaw_b = qOrg[5]
    wristRoll_b = qOrg[6]
    wristYaw2_b = qOrg[7]
	else
	  -- Two solutions
	  wristYaw_a = atan2(rotWrist[3][1], rotWrist[2][1])
	  wristYaw2_a = atan2(rotWrist[1][3], -rotWrist[1][2])
	  -- Flipped position
		-- -pi to 0
	  wristRoll_b = -wristRoll_a
	  wristYaw_b = atan2(-rotWrist[3][1], -rotWrist[2][1])
	  wristYaw2_b = atan2(-rotWrist[1][3], rotWrist[1][2])
  end
  local err_a = ( (qOrg[5] - wristYaw_a+5*PI) % TWO_PI ) - PI
  local err_b = ( (qOrg[5] - wristYaw_b+5*PI) % TWO_PI ) - PI
  if err_a^2 < err_b^2 then
    qArm[5] = wristYaw_a
    qArm[6] = wristRoll_a
    qArm[7] = wristYaw2_a
  else
    qArm[5] = wristYaw_b
    qArm[6] = wristRoll_b
    qArm[7] = wristYaw2_b
  end
	return qArm
  --return vnew(qArm)
end

-- Mounting Transform offsets
-- Left: Assume UCLA gripper
local preLArm, postLArm = Ttrans(shoulderOffsetX, shoulderOffsetY, shoulderOffsetZ), Ttrans(handOffsetX, -handOffsetY, handOffsetZ)
-- * Ttrans(0.045,0,0) * TrotZ(-45*DEG_TO_RAD) -- Add translation to the center of the palm, or what?
-- Right: UCLA Gripper
local preRArm, postRArm = Ttrans(shoulderOffsetX, -shoulderOffsetY, shoulderOffsetZ), Ttrans(handOffsetX, handOffsetY, handOffsetZ)
--* Ttrans(0.08,0,0) * TrotZ(45*DEG_TO_RAD)
-- Forward with respect to the torso
function K.forward_larm(qLArm)
	return preLArm * fk_arm(qLArm) * postLArm, {qLArm[3]}
end

function K.forward_rarm(qRArm)
	return preRArm * fk_arm(qRArm) * postRArm, {qRArm[3]}
end

-- Inverse with respect to the torso
local preLArmInv, postLArmInv = Tinv(preLArm), Tinv(postLArm)
function K.inverse_larm(trL, qLArm, shoulderYaw, flipRoll)
	return ik_arm(
		preLArmInv * trL * postLArmInv,
		qLArm,
		shoulderYaw or qLArm[3],
		flipRoll==1 and PI
	)
end
local preRArmInv, postRArmInv = Tinv(preRArm), Tinv(postRArm)
function K.inverse_rarm(trR, qRArm, shoulderYaw, flipRoll)
	return ik_arm(
		preRArmInv * trR * postRArmInv,
		qRArm,
		shoulderYaw or qRArm[3],
		flipRoll==1 and -PI
	)
end

-- Left leg based. Same for right for DH params, anyway
local function fk_leg(q)
	local c1, s1 = cos(q[1]), sin(q[1])
	local c2, s2 = cos(q[2]), sin(q[2])
	local c6, s6 = cos(q[6]), sin(q[6])
	local q3, q4, q5 = unpack(q,3,5)
	return Tnew{
		{((-(-s1*s2*sin(aThigh + q3) + c1*cos(aThigh + q3))*sin(aThigh + aTibia - q4) + (s1*s2*cos(aThigh + q3) + sin(aThigh + q3)*c1)*cos(aThigh + aTibia - q4))*cos(aTibia + q5) + ((-s1*s2*sin(aThigh + q3) + c1*cos(aThigh + q3))*cos(aThigh + aTibia - q4) + (s1*s2*cos(aThigh + q3) + sin(aThigh + q3)*c1)*sin(aThigh + aTibia - q4))*sin(aTibia + q5))*c6 + s1*s6*c2, -((-(-s1*s2*sin(aThigh + q3) + c1*cos(aThigh + q3))*sin(aThigh + aTibia - q4) + (s1*s2*cos(aThigh + q3) + sin(aThigh + q3)*c1)*cos(aThigh + aTibia - q4))*cos(aTibia + q5) + ((-s1*s2*sin(aThigh + q3) + c1*cos(aThigh + q3))*cos(aThigh + aTibia - q4) + (s1*s2*cos(aThigh + q3) + sin(aThigh + q3)*c1)*sin(aThigh + aTibia - q4))*sin(aTibia + q5))*s6 + s1*c2*c6, -(-(-s1*s2*sin(aThigh + q3) + c1*cos(aThigh + q3))*sin(aThigh + aTibia - q4) + (s1*s2*cos(aThigh + q3) + sin(aThigh + q3)*c1)*cos(aThigh + aTibia - q4))*sin(aTibia + q5) + ((-s1*s2*sin(aThigh + q3) + c1*cos(aThigh + q3))*cos(aThigh + aTibia - q4) + (s1*s2*cos(aThigh + q3) + sin(aThigh + q3)*c1)*sin(aThigh + aTibia - q4))*cos(aTibia + q5), -dThigh*(s1*s2*cos(aThigh + q3) + sin(aThigh + q3)*c1) - dTibia*(-(-s1*s2*sin(aThigh + q3) + c1*cos(aThigh + q3))*sin(aThigh + aTibia - q4) + (s1*s2*cos(aThigh + q3) + sin(aThigh + q3)*c1)*cos(aThigh + aTibia - q4))},
		{(((s1*sin(aThigh + q3) - s2*c1*cos(aThigh + q3))*sin(aThigh + aTibia - q4) + (s1*cos(aThigh + q3) + s2*sin(aThigh + q3)*c1)*cos(aThigh + aTibia - q4))*sin(aTibia + q5) + ((s1*sin(aThigh + q3) - s2*c1*cos(aThigh + q3))*cos(aThigh + aTibia - q4) - (s1*cos(aThigh + q3) + s2*sin(aThigh + q3)*c1)*sin(aThigh + aTibia - q4))*cos(aTibia + q5))*c6 - s6*c1*c2, -(((s1*sin(aThigh + q3) - s2*c1*cos(aThigh + q3))*sin(aThigh + aTibia - q4) + (s1*cos(aThigh + q3) + s2*sin(aThigh + q3)*c1)*cos(aThigh + aTibia - q4))*sin(aTibia + q5) + ((s1*sin(aThigh + q3) - s2*c1*cos(aThigh + q3))*cos(aThigh + aTibia - q4) - (s1*cos(aThigh + q3) + s2*sin(aThigh + q3)*c1)*sin(aThigh + aTibia - q4))*cos(aTibia + q5))*s6 - c1*c2*c6, ((s1*sin(aThigh + q3) - s2*c1*cos(aThigh + q3))*sin(aThigh + aTibia - q4) + (s1*cos(aThigh + q3) + s2*sin(aThigh + q3)*c1)*cos(aThigh + aTibia - q4))*cos(aTibia + q5) - ((s1*sin(aThigh + q3) - s2*c1*cos(aThigh + q3))*cos(aThigh + aTibia - q4) - (s1*cos(aThigh + q3) + s2*sin(aThigh + q3)*c1)*sin(aThigh + aTibia - q4))*sin(aTibia + q5), -dThigh*(s1*sin(aThigh + q3) - s2*c1*cos(aThigh + q3)) - dTibia*((s1*sin(aThigh + q3) - s2*c1*cos(aThigh + q3))*cos(aThigh + aTibia - q4) - (s1*cos(aThigh + q3) + s2*sin(aThigh + q3)*c1)*sin(aThigh + aTibia - q4))},
		{((sin(aThigh + q3)*sin(aThigh + aTibia - q4)*c2 + c2*cos(aThigh + q3)*cos(aThigh + aTibia - q4))*cos(aTibia + q5) + (-sin(aThigh + q3)*c2*cos(aThigh + aTibia - q4) + sin(aThigh + aTibia - q4)*c2*cos(aThigh + q3))*sin(aTibia + q5))*c6 - s2*s6, -((sin(aThigh + q3)*sin(aThigh + aTibia - q4)*c2 + c2*cos(aThigh + q3)*cos(aThigh + aTibia - q4))*cos(aTibia + q5) + (-sin(aThigh + q3)*c2*cos(aThigh + aTibia - q4) + sin(aThigh + aTibia - q4)*c2*cos(aThigh + q3))*sin(aTibia + q5))*s6 - s2*c6, -(sin(aThigh + q3)*sin(aThigh + aTibia - q4)*c2 + c2*cos(aThigh + q3)*cos(aThigh + aTibia - q4))*sin(aTibia + q5) + (-sin(aThigh + q3)*c2*cos(aThigh + aTibia - q4) + sin(aThigh + aTibia - q4)*c2*cos(aThigh + q3))*cos(aTibia + q5), -dThigh*c2*cos(aThigh + q3) - dTibia*(sin(aThigh + q3)*sin(aThigh + aTibia - q4)*c2 + c2*cos(aThigh + q3)*cos(aThigh + aTibia - q4))},
		{0, 0, 0, 1}}
end

local preLLeg, postLLeg = Ttrans(0, hipOffsetY, -hipOffsetZ), TrotZ(PI) * TrotY(-PI/2) * Ttrans(0,0,-footHeight)
function K.forward_lleg(qLLeg)
	return preLLeg * fk_leg(qLLeg) * postLLeg
end

local preRLeg, postRLeg = Ttrans(0, -hipOffsetY, -hipOffsetZ), TrotZ(PI) * TrotY(-PI/2) * Ttrans(0,0,-footHeight)
function K.forward_rleg(qRLeg)
	return preRLeg * fk_leg(qRLeg) * postRLeg
end

-- NOTE: Allow the ankle tilt, as well
-- From a5e23ca190258b7957ff18932055e4baaca19346
local function ik_leg(trLeg, hipOffset)
  -- Hip Offset vector in Torso frame
	local xLeg1, xLeg2, xLeg3 = unpack(Tinv(trLeg) * hipOffset)
  xLeg3 = xLeg3 - footHeight
  local dLeg = xLeg1^2 + xLeg2^2 + xLeg3^2
	-- Grab the knee angle
  local cKnee = (dLeg - dTibia^2 - dThigh^2)/(2 * dTibia * dThigh)
	local kneePitch = acos(max(-1, min(1, cKnee)))
	-- Ankle pitch and roll
	local ankleRoll = atan2(xLeg2, xLeg3)
	local lLeg = max(1e-16, sqrt(dLeg))
	local pitch0 = asin(dThigh * sin(kneePitch) / lLeg)
	local anklePitch = asin(-xLeg1 / lLeg) - pitch0
	-- Hip
	local rHipT = trLeg * TrotX(-ankleRoll) * TrotY(-anklePitch - kneePitch)
	local hipYaw = atan2(-rHipT[1][2], rHipT[2][2])
	local hipRoll = asin(rHipT[3][2])
	local hipPitch = atan2(-rHipT[3][1], rHipT[3][3])
  -- NOTE: Need to compensate for KneeOffsetX:
	return vnew{hipYaw, hipRoll, hipPitch-aThigh, kneePitch + aThigh + aTibia, anklePitch-aTibia, ankleRoll}
end

local offsetLHip = {0, hipOffsetY, -hipOffsetZ}
function K.inverse_lleg(trLLeg)
	return ik_leg(trLLeg, offsetLHip)
end

local offsetRHip = {0, -hipOffsetY, -hipOffsetZ}
function K.inverse_rleg(trRLeg)
	return ik_leg(trRLeg, offsetRHip)
end

function K.inverse_legs(trLLeg, trRLeg, trTorso)
	local invTorso = Tinv(trTorso)
	return ik_leg(invTorso*trLLeg, offsetLHip), ik_leg(invTorso*trRLeg, offsetRHip)
end

return K
