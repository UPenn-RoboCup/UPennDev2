local K = {}
local vector = require'vector'
local torch = require'torch'
local T = require'libTransform'
local T0 = require'Transform'
local Kinematics = require'THOROPKinematics'
-- Cache math
local sin, cos = math.sin, math.cos
local asin, acos = math.asin, math.acos
local sqrt, atan2 = math.sqrt, math.atan2
local PI = math.pi

local shoulderOffsetX = 0;    
local shoulderOffsetY = 0.234;
local shoulderOffsetZ = 0.165;
local upperArmLength = .246;
local elbowOffsetX =   .030; 
--local lowerArmLength = .186; -- Default 7DOF arm
local lowerArmLength = .250; -- LONGARM model
-- No appendage - just the plate
local handOffsetX = 0.245;
local handOffsetY = 0.035;
local handOffsetZ = 0;
local trPlate = T.trans(handOffsetX, handOffsetY, handOffsetZ)
-- TODO: Add the 45 Degree change here
--local trGripper = trPlate * T.trans(0.025, 0, 0) * T.rotZ(45*DEG_TO_RAD)
-- Shoulder (TODO: Left/Right?)
-- TODO: Add the waist with this...
local trShoulder = T.trans(shoulderOffsetX, shoulderOffsetY, shoulderOffsetZ)

local function fk_arm(q)
	local c1, s1 = cos(q[1]), sin(q[1])
	local c2, s2 = cos(q[2]), sin(q[2])
	local c3, s3 = cos(q[3]), sin(q[3])
	local c4, s4 = cos(q[4]), sin(q[4])
	local c5, s5 = cos(q[5]), sin(q[5])
	local c6, s6 = cos(q[6]), sin(q[6])
	local c7, s7 = cos(q[7]), sin(q[7])
	return T0.new{{(((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*s5 + (s1*s3 - s2*c1*c3)*c5)*s6 + (-(s1*c3 + s2*s3*c1)*s4 + c1*c2*c4)*c6, ((((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*s5 + (s1*s3 - s2*c1*c3)*c5)*c6 - (-(s1*c3 + s2*s3*c1)*s4 + c1*c2*c4)*s6)*c7 + (((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*c5 - (s1*s3 - s2*c1*c3)*s5)*s7, -((((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*s5 + (s1*s3 - s2*c1*c3)*c5)*c6 - (-(s1*c3 + s2*s3*c1)*s4 + c1*c2*c4)*s6)*s7 + (((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*c5 - (s1*s3 - s2*c1*c3)*s5)*c7, -0.25*(s1*c3 + s2*s3*c1)*s4 - 0.03*(s1*c3 + s2*s3*c1)*c4 + 0.03*s1*c3 + 0.03*s2*s3*c1 - 0.03*s4*c1*c2 + 0.25*c1*c2*c4 + 0.246*c1*c2}, {((s2*s4 - s3*c2*c4)*s5 + c2*c3*c5)*s6 + (s2*c4 + s3*s4*c2)*c6, (((s2*s4 - s3*c2*c4)*s5 + c2*c3*c5)*c6 - (s2*c4 + s3*s4*c2)*s6)*c7 + ((s2*s4 - s3*c2*c4)*c5 - s5*c2*c3)*s7, -(((s2*s4 - s3*c2*c4)*s5 + c2*c3*c5)*c6 - (s2*c4 + s3*s4*c2)*s6)*s7 + ((s2*s4 - s3*c2*c4)*c5 - s5*c2*c3)*c7, -0.03*s2*s4 + 0.25*s2*c4 + 0.246*s2 + 0.25*s3*s4*c2 + 0.03*s3*c2*c4 - 0.03*s3*c2}, {(((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*s5 + (s1*s2*c3 + s3*c1)*c5)*s6 + (-(-s1*s2*s3 + c1*c3)*s4 - s1*c2*c4)*c6, ((((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*s5 + (s1*s2*c3 + s3*c1)*c5)*c6 - (-(-s1*s2*s3 + c1*c3)*s4 - s1*c2*c4)*s6)*c7 + (((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*c5 - (s1*s2*c3 + s3*c1)*s5)*s7, -((((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*s5 + (s1*s2*c3 + s3*c1)*c5)*c6 - (-(-s1*s2*s3 + c1*c3)*s4 - s1*c2*c4)*s6)*s7 + (((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*c5 - (s1*s2*c3 + s3*c1)*s5)*c7, -0.25*(-s1*s2*s3 + c1*c3)*s4 - 0.03*(-s1*s2*s3 + c1*c3)*c4 - 0.03*s1*s2*s3 + 0.03*s1*s4*c2 - 0.25*s1*c2*c4 - 0.246*s1*c2 + 0.03*c1*c3}, {0, 0, 0, 1}}
end
K.forward_arm = fk_arm

local function fk_arm_t(q)
	local c1, s1 = cos(q[1]), sin(q[1])
	local c2, s2 = cos(q[2]), sin(q[2])
	local c3, s3 = cos(q[3]), sin(q[3])
	local c4, s4 = cos(q[4]), sin(q[4])
	local c5, s5 = cos(q[5]), sin(q[5])
	local c6, s6 = cos(q[6]), sin(q[6])
	local c7, s7 = cos(q[7]), sin(q[7])
	local tr = torch.Tensor(4,4)
	local tr_d = tr:data()
	tr_d[0] = (((s1*c3+s2*s3*c1)*c4+s4*c1*c2)*s5+(s1*s3-s2*c1*c3)*c5)*s6+(-(s1*c3+s2*s3*c1)*s4+c1*c2*c4)*c6
	tr_d[1] = ((((s1*c3+s2*s3*c1)*c4+s4*c1*c2)*s5+(s1*s3-s2*c1*c3)*c5)*c6-(-(s1*c3+s2*s3*c1)*s4+c1*c2*c4)*s6)*c7+(((s1*c3+s2*s3*c1)*c4+s4*c1*c2)*c5-(s1*s3-s2*c1*c3)*s5)*s7
	tr_d[2] = -((((s1*c3+s2*s3*c1)*c4+s4*c1*c2)*s5+(s1*s3-s2*c1*c3)*c5)*c6-(-(s1*c3+s2*s3*c1)*s4+c1*c2*c4)*s6)*s7+(((s1*c3+s2*s3*c1)*c4+s4*c1*c2)*c5-(s1*s3-s2*c1*c3)*s5)*c7
	tr_d[3] = -0.25*(s1*c3+s2*s3*c1)*s4-0.03*(s1*c3+s2*s3*c1)*c4+0.03*s1*c3+0.03*s2*s3*c1-0.03*s4*c1*c2+0.25*c1*c2*c4+0.246*c1*c2
	tr_d[4] = ((s2*s4-s3*c2*c4)*s5+c2*c3*c5)*s6+(s2*c4+s3*s4*c2)*c6
	tr_d[5] = (((s2*s4-s3*c2*c4)*s5+c2*c3*c5)*c6-(s2*c4+s3*s4*c2)*s6)*c7+((s2*s4-s3*c2*c4)*c5-s5*c2*c3)*s7
	tr_d[6] = -(((s2*s4-s3*c2*c4)*s5+c2*c3*c5)*c6-(s2*c4+s3*s4*c2)*s6)*s7+((s2*s4-s3*c2*c4)*c5-s5*c2*c3)*c7
	tr_d[7] = -0.03*s2*s4+0.25*s2*c4+0.246*s2+0.25*s3*s4*c2+0.03*s3*c2*c4-0.03*s3*c2
	tr_d[8] = (((-s1*s2*s3+c1*c3)*c4-s1*s4*c2)*s5+(s1*s2*c3+s3*c1)*c5)*s6+(-(-s1*s2*s3+c1*c3)*s4-s1*c2*c4)*c6
	tr_d[9] = ((((-s1*s2*s3+c1*c3)*c4-s1*s4*c2)*s5+(s1*s2*c3+s3*c1)*c5)*c6-(-(-s1*s2*s3+c1*c3)*s4-s1*c2*c4)*s6)*c7+(((-s1*s2*s3+c1*c3)*c4-s1*s4*c2)*c5-(s1*s2*c3+s3*c1)*s5)*s7
	tr_d[10] = -((((-s1*s2*s3+c1*c3)*c4-s1*s4*c2)*s5+(s1*s2*c3+s3*c1)*c5)*c6-(-(-s1*s2*s3+c1*c3)*s4-s1*c2*c4)*s6)*s7+(((-s1*s2*s3+c1*c3)*c4-s1*s4*c2)*c5-(s1*s2*c3+s3*c1)*s5)*c7
	tr_d[11] = -0.25*(-s1*s2*s3+c1*c3)*s4-0.03*(-s1*s2*s3+c1*c3)*c4-0.03*s1*s2*s3+0.03*s1*s4*c2-0.25*s1*c2*c4-0.246*s1*c2+0.03*c1*c3
	tr_d[12] = 0
	tr_d[13] = 0
	tr_d[14] = 0
	tr_d[15] = 1
	return tr
end
K.forward_arm_t = fk_arm_t

-- Precalculate some stuff
local trans_upper = T0.trans(upperArmLength, 0, elbowOffsetX)
local trans_lower = T0.trans(lowerArmLength, 0, -elbowOffsetX)
local dUpperArm = sqrt(upperArmLength^2 + elbowOffsetX^2)
local dLowerArm = sqrt(lowerArmLength^2 + elbowOffsetX^2)
local aUpperArm = math.atan(elbowOffsetX / upperArmLength)
local aLowerArm = math.atan(elbowOffsetX / lowerArmLength)
local aElbowMax = -1*(aUpperArm + aLowerArm)
-- Assume left arm
local function ik_arm2(trArm, qOrg, shoulderYaw, FLIP_SHOULDER_ROLL)
  trArm = T0.copy(trArm)
  local xWrist = {trArm[1][4], trArm[2][4], trArm[3][4]}
  
  -- SJ: Robot can have TWO elbow pitch values (near elbowPitch==0)
  -- We are only using the smaller one (arm more bent) 
  local dWrist = xWrist[1]^2 + xWrist[2]^2 + xWrist[3]^2
  local cElbow = (dWrist-dUpperArm^2 - dLowerArm^2)/(2*dUpperArm*dLowerArm)
  if (cElbow > 1) then cElbow = 1 elseif (cElbow < -1) then cElbow = -1 end
  local elbowPitch = -acos(cElbow)-aUpperArm-aLowerArm
  
  -- From shoulder yaw to wrist 
  local m = T0.rotX(shoulderYaw) * trans_upper * T0.rotY(elbowPitch) * trans_lower
  
  local a = m[1][4]^2 + m[2][4]^2
  local b = -m[1][4] * xWrist[2]
  local c = xWrist[2]^2 - m[2][4]^2
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
    if (s21 > 1) then s21 = 1 elseif (s21 < -1) then s21 = -1 end
    if (s22 > 1) then s22 = 1 elseif (s22 < -1) then s22 = -1 end
    local shoulderRoll1, shoulderRoll2 = asin(s21), asin(s22)
    local err1 = s21*m[1][4] + cos(shoulderRoll1)*m[2][4] - xWrist[2]
    local err2 = s22*m[1][4] + cos(shoulderRoll2)*m[2][4] - xWrist[2]
    shoulderRoll = err1^2 < err2^2 and shoulderRoll1 or shoulderRoll2
  end
  if FLIP_SHOULDER_ROLL then
    shoulderRoll = PI - shoulderRoll
  end
  
  local s2 = sin(shoulderRoll)
  local c2 = cos(shoulderRoll)
  local t1 = -c2 * m[1][4] + s2 * m[2][4]
 
  local m23 = m[3][4]
  local c1 = (m23*xWrist[3] - t1*xWrist[1]) / (m23^2 + t1^2)
  local s1 = (m23*xWrist[1] + t1*xWrist[3]) / (m23^2 + t1^2)

  local shoulderPitch = atan2(s1, c1)
	
	local qArm = {shoulderPitch, shoulderRoll, shoulderYaw, elbowPitch}
	
	--
	-- Now find the wrist
	--
	local tRot = T0.rotY(shoulderPitch) * T0.rotZ(shoulderRoll) * T0.rotX(shoulderYaw) * T0.rotY(elbowPitch)
	local tInvRot = T0.inv(tRot)
	
  
  -- Now we know shoulder pich, roll, yaw and elbow pitch
  -- Calc the final transform for the wrist based on rotation alone
  local rotWrist = tInvRot * trArm
  
  -- Two solutions
  local wristRoll_a = acos(rotWrist[1][1]) -- 0 to pi
  local swa = sin(wristRoll_a)
  local wristYaw_a = atan2 (rotWrist[3][1]*swa,rotWrist[2][1]*swa)
  local wristYaw2_a = atan2 (rotWrist[1][3]*swa,-rotWrist[1][2]*swa)

  -- Flipped position
  local wristRoll_b = -wristRoll_a -- -pi to 0
  local swb = sin(wristRoll_b)
  local wristYaw_b = atan2(rotWrist[3][1]*swb, rotWrist[2][1]*swb)  
  local wristYaw2_b = atan2(rotWrist[1][3]*swb, -rotWrist[1][2]*swb)
  -- singular point: just use current angles
  if swa<1e-5 then
    wristYaw_a = qOrg[5]    
    wristRoll_a = qOrg[6]
    wristYaw2_a = qOrg[7]
    wristYaw_b = qOrg[5]
    wristRoll_b = qOrg[6]
    wristYaw2_b = qOrg[7]
  end

  local err_a = ( (qOrg[5] - wristYaw_a+5*PI) % 2*PI ) - PI
  local err_b = ( (qOrg[5] - wristYaw_b+5*PI) % 2*PI ) - PI
  if err_a^2 < err_b^2 then
    qArm[5] = wristYaw_a
    qArm[6] = wristRoll_a
    qArm[7] = wristYaw2_a
  else
    qArm[5] = wristYaw_b
    qArm[6] = wristRoll_b
    qArm[7] = wristYaw2_b
  end
  return vector.new(qArm)
end
K.inverse_arm2 = ik_arm2

local ik_arm = Kinematics.inverse_arm
K.inverse_arm = ik_arm

-- Forward with respect to the torso
function K.forward_l_arm(qLArm)
	--[[
	local tr0 = fk_arm(qLArm)
	tr0[1][4] = tr0[1][4] + shoulderOffsetX
	tr0[2][4] = tr0[2][4] + shoulderOffsetY
	tr0[3][4] = tr0[3][4] + shoulderOffsetZ
	return T.copy(tr0), {qLArm[3]}
	--]]
	--[[
	local tr0 = Kinematics.l_arm_torso_7(qRArm, 0, {0,0}, 0,0,0)
	return T.transform6D(tr0), {qLArm[3]}
	--]]
	--[[
	local tr0 = T0.trans(shoulderOffsetX, shoulderOffsetY, shoulderOffsetZ) * fk_arm(qLArm) * T0.trans(handOffsetX, handOffsetY, handOffsetZ)
	--]]
	local tr0 = Kinematics.l_arm_torso_7(qLArm, 0, {0,0})
	return T.transform6D(tr0), {qLArm[3]}	
end
function K.forward_r_arm(qRArm)
	--[[
	local tr0 = fk_arm(qRArm)	
	tr0[1][4] = tr0[1][4] + shoulderOffsetX
	tr0[2][4] = tr0[2][4] - shoulderOffsetY
	tr0[3][4] = tr0[3][4] + shoulderOffsetZ
	return T.copy(tr0), {qRArm[3]}
	--]]
	--[[
	local tr0 = Kinematics.r_arm_torso_7(qRArm, 0, {0,0}, 0,0,0)
	print(T.transform6D(tr0))
	return T.transform6D(tr0), {qRArm[3]}
	--]]
	--[[
	local tr0 = T0.trans(shoulderOffsetX, -shoulderOffsetY, shoulderOffsetZ) * fk_arm(qRArm) * T0.trans(handOffsetX, handOffsetY, handOffsetZ)
	--]]
	local tr0 = Kinematics.r_arm_torso_7(qRArm, 0, {0,0})
	return T.transform6D(tr0), {qRArm[3]}	
end

-- Inverse with respect to the torso
function K.inverse_l_arm(trL, qLArm, shoulderYaw, flipRoll)
	----[[
	local tr = T.copy(trL)
	tr[1][4] = tr[1][4] - shoulderOffsetX
	tr[2][4] = tr[2][4] - shoulderOffsetY
	tr[3][4] = tr[3][4] - shoulderOffsetZ
	--return vector.new(ik_arm(tr, qLArm or vector.zeros(7), shoulderYaw or qLArm[3], flipRoll))
	print('ik_arm1', vector.new(ik_arm(tr, qLArm or vector.zeros(7), shoulderYaw or qLArm[3], flipRoll)))
	--]]
	----[[
	local t0 = unix.time()
	local tr6 = vector.new(T.position6D(trL))
	local sol1 = vector.new(Kinematics.inverse_l_arm_7(tr6, qLArm, shoulderYaw or qLArm[3], 0, {0,0}, 0,0,0))
	local t1 = unix.time()
	print(t1-t0, 'ik_armC',sol1)
	--return vector.new(Kinematics.inverse_l_arm_7(tr6, qLArm, shoulderYaw or qLArm[3], 0, {0,0}, 0,0,0))
	--]]
	
	local t0 = unix.time()
	local sol2 = ik_arm2(tr, qLArm, shoulderYaw or qLArm[3])
	local t1 = unix.time()
	print(t1-t0, 'ik_arm2', sol2)
	
	local tr6 = T0.position6D(trL)
	return vector.new(Kinematics.inverse_l_arm_7(tr6, qLArm, shoulderYaw or qLArm[3], 0, {0,0}))
end
function K.inverse_r_arm(trR, qRArm, shoulderYaw, flipRoll)
	--[[
	local tr = T.copy(trR)
	tr[1][4] = tr[1][4] - shoulderOffsetX
	tr[2][4] = tr[2][4] + shoulderOffsetY
	tr[3][4] = tr[3][4] - shoulderOffsetZ
	return vector.new(ik_arm(tr, qRArm or vector.zeros(7), shoulderYaw or qRArm[3], flipRoll))
	--]]
	--[[
	local tr6 = vector.new(T.position6D(trR))
	return vector.new(Kinematics.inverse_r_arm_7(tr6, qRArm, shoulderYaw or qRArm[3], 0, {0,0}, 0,0,0))
	--]]
	local tr6 = T0.position6D(trR)
	return vector.new(Kinematics.inverse_r_arm_7(tr6, qRArm, shoulderYaw or qRArm[3], 0, {0,0}))
end

return K
