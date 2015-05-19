local K = {}
--local T = require'libTransform'
local T = require'Transform'
local vector = require'vector'
local ok, ffi = pcall(require, 'ffi')
local ptorch = require'util'.ptorch
local sin, cos = math.sin, math.cos
local asin, acos = math.asin, math.acos
local sqrt, atan2 = math.sqrt, math.atan2
local PI = math.pi
local shoulderOffsetX = 0    
local shoulderOffsetY = 0.234
local shoulderOffsetZ = 0.165
local upperArmLength = .246
local elbowOffsetX =   .030 
--local lowerArmLength = .186 //Default 7DOF arm
local lowerArmLength = .250 -- LONGARM model
local dUpperArm = sqrt(upperArmLength*upperArmLength+elbowOffsetX*elbowOffsetX)
local dLowerArm = sqrt(lowerArmLength*lowerArmLength+elbowOffsetX*elbowOffsetX)
local aUpperArm = math.atan(elbowOffsetX/upperArmLength)
local aLowerArm = math.atan(elbowOffsetX/lowerArmLength)
local aElbowMax = -1*(aUpperArm + aLowerArm)
--print('aElbowMax', aElbowMax, aElbowMax * RAD_TO_DEG)

-- Precalculate some stuff
local trans_upper = T.trans(upperArmLength, 0, elbowOffsetX)
local trans_lower = T.trans(lowerArmLength,0,-elbowOffsetX)

-- trArm: Desired transform of the arm. Assume body rotations are removed

local function ik(trArm, qOrg, shoulderYaw, FLIP_SHOULDER_ROLL)
  trArm = T.copy(trArm)

  -- TODO: Use the cdata directly, rather than in and out of C
  local xWrist = {trArm[1][4], trArm[2][4], trArm[3][4]}
  
  -- SJ: Robot can have TWO elbow pitch values (near elbowPitch==0)
  -- We are only using the smaller one (arm more bent) 
  local dWrist = xWrist[1]^2+xWrist[2]^2+xWrist[3]^2
  local cElbow = (dWrist-dUpperArm^2-dLowerArm^2)/(2*dUpperArm*dLowerArm)
  if (cElbow > 1) then cElbow = 1 elseif (cElbow < -1) then cElbow = -1 end
  local elbowPitch = aElbowMax - acos(cElbow)

  
  -- From shoulder yaw to wrist 
  local m = T.rotX(shoulderYaw) * trans_upper * T.rotY(elbowPitch) * trans_lower
  
  local a = m[1][4]^2 + m[2][4]^2
  local b = -m[1][4] * xWrist[2]
  local c = xWrist[2]^2 - m[2][4]^2
  --
  local shoulderRoll
  -- NaN handling
  if (b*b-a*c)<0 or a==0 then
    shoulderRoll = 0
  else
    local s21 = (-b+sqrt(b*b-a*c))/a
    local s22 = (-b-sqrt(b*b-a*c))/a
    if (s21 > 1) then s21 = 1 elseif (s21 < -1) then s21 = -1 end
    if (s22 > 1) then s22 = 1 elseif (s22 < -1) then s22 = -1 end
    local shoulderRoll1 = asin(s21)
    local shoulderRoll2 = asin(s22)
    local err1 = s21*m[1][4]+cos(shoulderRoll1)*m[2][4]-xWrist[2]
    local err2 = s22*m[1][4]+cos(shoulderRoll2)*m[2][4]-xWrist[2]
    shoulderRoll = err1^2<err2^2 and shoulderRoll1 or shoulderRoll2
  end
  if FLIP_SHOULDER_ROLL then
    shoulderRoll = (arm==ARM_LEFT) and PI - shoulderRoll or (-PI) - shoulderRoll
  end
  
  local s2 = sin(shoulderRoll)
  local c2 = cos(shoulderRoll)
  local t1 = -c2 * m[1][4] + s2 * m[2][4]
 
  local m23 = m[3][4]
  local c1 = (m23*xWrist[3]-t1*xWrist[1]) /(m23^2 + t1^2)
  local s1 = (m23*xWrist[1]+t1*xWrist[3]) /(m23^2 + t1^2)

  local shoulderPitch = atan2(s1,c1)
  
  -- Now we know shoulder pich, roll, yaw and elbow pitch
  -- Calc the final transform for the wrist based on rotation alone
  local rotWrist = T.rotY(-shoulderPitch) * T.rotZ(-shoulderRoll) * T.rotX(-shoulderYaw) * T.rotY(-elbowPitch) * trArm
  
  -- Two solutions
  local wristRoll_a = acos(rotWrist[1][1]) -- 0 to pi
  local swa = sin(wristRoll_a)
  local wristYaw_a = atan2 (rotWrist[3][1]*swa,rotWrist[2][1]*swa)
  local wristYaw2_a = atan2 (rotWrist[1][3]*swa,-rotWrist[1][2]*swa)

  -- Flipped position
  local wristRoll_b = -wristRoll_a -- -pi to 0
  local swb = sin(wristRoll_b)
  local wristYaw_b = atan2 (rotWrist[3][1]*swb,rotWrist[2][1]*swb)  
  local wristYaw2_b = atan2 (rotWrist[1][3]*swb,-rotWrist[1][2]*swb)
  -- singular point: just use current angles
  if swa<1e-5 then
    wristYaw_a = qOrg[5]    
    wristRoll_a = qOrg[6]
    wristYaw2_a = qOrg[7]
    wristYaw_b = qOrg[5]
    wristRoll_b = qOrg[6]
    wristYaw2_b = qOrg[7]
  end
  
  local select_a = false
  local err_a = ( (qOrg[5] - wristYaw_a+5*PI) % 2*PI ) - PI
  local err_b = ( (qOrg[5] - wristYaw_b+5*PI) % 2*PI ) - PI
  if (err_a*err_a<err_b*err_b) then select_a=true end
  
  local qArm = {shoulderPitch, shoulderRoll, shoulderYaw, elbowPitch}

  if (select_a==true) then
    qArm[5] = wristYaw_a
    qArm[6] = wristRoll_a
    qArm[7] = wristYaw2_a
  else
    qArm[5] = wristYaw_b
    qArm[6] = wristRoll_b
    qArm[7] = wristYaw2_b
  end
  return qArm
end

local xWrist = ffi.new('double[3]')
local m = ffi.new('double[4][4]')
local rotWrist = ffi.new('double[4][4]')
local rX = ffi.new('double[4][4]', T.eye())
local rY = ffi.new('double[4][4]', T.eye())
local rZ = ffi.new('double[4][4]', T.eye())
local tUpper = ffi.new('double[4][4]', {
  {1, 0, 0, upperArmLength},
  {0, 1, 0, 0},
  {0, 0, 1, elbowOffsetX},
  {0, 0, 0, 1}
})
local tLower = ffi.new('double[4][4]', {
  {1, 0, 0, lowerArmLength},
  {0, 1, 0, 0},
  {0, 0, 1, -elbowOffsetX},
  {0, 0, 0, 1}
})

local function rotX(a)
  local ca = cos(a)
  local sa = sin(a)
  rX[1][1] = ca
  rX[2][2] = ca
  rX[1][2] = -sa
  rX[2][1] = sa
  return rX
end

local function rotY(a)
  local ca = cos(a)
  local sa = sin(a)
  rY[0][0] = ca
  rY[2][2] = ca
  rY[0][2] = sa
  rY[2][0] = -sa
  return rY
end

local function rotZ(a)
  local ca = cos(a)
  local sa = sin(a)
  rZ[0][0] = ca
  rZ[1][1] = ca
  rZ[0][1] = -sa
  rZ[1][0] = sa
  return rZ
end

local tmp3 = ffi.new('double[4][4]')
local tmp2 = ffi.new('double[4][4]')
local tmp1 = ffi.new('double[4][4]')
local function mul(res, t1, t2)
  for i = 0,3 do
    for j = 0,3 do
      res[i][j] = t1[i][0] * t2[0][j] + t1[i][1] * t2[1][j] + t1[i][2] * t2[2][j] + t1[i][3] * t2[3][j]
    end
  end
  return res
end

local elbow_tmp1, elbow_tmp2 = dUpperArm^2 + dLowerArm^2, 2*dUpperArm*dLowerArm

local PI_5, PI_2, P_N = PI * 5, 2 * PI, -PI

local function ik2(trArm, qOrg, shoulderYaw, FLIP_SHOULDER_ROLL)
  -- trArm is what?
  
  -- Grab the position
  xWrist[0] = trArm[0][3]
  xWrist[1] = trArm[1][3]
  xWrist[2] = trArm[2][3]
  
  local cElbow = (xWrist[0]^2+xWrist[1]^2+xWrist[2]^2 - elbow_tmp1) / elbow_tmp2
  local aElbow = acos(cElbow > 1 and cElbow or cElbow < -1 and -1 or cElbow)
  -- If less bent already, then use positive acos
  local elbowPitch = (qOrg[4] > aElbowMax and 1 or -1)*aElbow + aElbowMax
  
  -- From shoulder yaw to wrist
  mul(m,
    mul(tmp1,
      mul(tmp2,
        rotX(shoulderYaw), tUpper
      ),
      rotY(elbowPitch)
    ),
    tLower
  )
  
  local a = m[0][3]^2 + m[1][3]^2
  local b = -m[0][3] * xWrist[1]
  local c = xWrist[0]^2 - m[1][3]^2
  --
  local shoulderRoll
  -- NaN handling
  local det = b^2 - a*c
  if det<0 or a==0 then
    shoulderRoll = 0
  else
    local s21 = (-b+sqrt(det))/a
    local s22 = (-b-sqrt(det))/a
    s21 = s21 > 1 and 1 or s21 < -1 and -1 or s21
    s22 = s22 > 1 and 1 or s22 < -1 and -1 or s22
    local shoulderRoll1 = asin(s21)
    local shoulderRoll2 = asin(s22)
    local err1 = s21 * m[0][3] + cos(shoulderRoll1) * m[1][3] - xWrist[1]
    local err2 = s22 * m[0][3] + cos(shoulderRoll2) * m[1][3] - xWrist[1]
    shoulderRoll = err1^2 < err2^2 and shoulderRoll1 or shoulderRoll2
  end
  if FLIP_SHOULDER_ROLL then
    shoulderRoll = (arm==ARM_LEFT) and PI - shoulderRoll or PI_N - shoulderRoll
  end
  
  local s2 = sin(shoulderRoll)
  local c2 = cos(shoulderRoll)
  local t1 = -c2 * m[0][3] + s2 * m[1][3]
 
  local m23 = m[2][3]
  local c1 = (m23 * xWrist[2] - t1 * xWrist[0]) / (m23^2 + t1^2)
  local s1 = (m23 * xWrist[0] + t1 * xWrist[2]) / (m23^2 + t1^2)

  local shoulderPitch = atan2(s1, c1)
  
  -- Now we know shoulder pich, roll, yaw and elbow pitch
  -- Calc the final transform for the wrist based on rotation alone
  mul(rotWrist,
    mul(tmp1,
      mul(tmp2,
        mul(tmp3,
          rotY(-shoulderPitch), rotZ(-shoulderRoll)
        ),
        rotX(-shoulderYaw)
      ),
      rotY(-elbowPitch)
    ),
    trArm
  )
    
  -- Two solutions
  local wristRoll_a = acos(rotWrist[0][0]) -- 0 to pi
  local swa = sin(wristRoll_a)
  local wristYaw_a = atan2(rotWrist[2][0] * swa, rotWrist[1][0] * swa)
  local wristYaw2_a = atan2(rotWrist[0][2] * swa, -rotWrist[0][1] * swa)

  -- Flipped position
  local wristRoll_b = -wristRoll_a -- -pi to 0
  local swb = sin(wristRoll_b)
  local wristYaw_b = atan2(rotWrist[2][0] * swb, rotWrist[1][0] * swb)  
  local wristYaw2_b = atan2(rotWrist[0][2] * swb, -rotWrist[0][1] * swb)
  
  -- singular point: just use current angles
  if swa<1e-5 then
    wristYaw_a = qOrg[5]    
    wristRoll_a = qOrg[6]
    wristYaw2_a = qOrg[7]
    wristYaw_b = qOrg[5]
    wristRoll_b = qOrg[6]
    wristYaw2_b = qOrg[7]
  end
  
  local err_a = ( (qOrg[5] - wristYaw_a + PI_5) % PI_2 ) - PI
  local err_b = ( (qOrg[5] - wristYaw_b + PI_5) % PI_2 ) - PI
  
  local qArm = {shoulderPitch, shoulderRoll, shoulderYaw, elbowPitch}

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
end

K.ik = ik
K.ik2 = ik2

return K