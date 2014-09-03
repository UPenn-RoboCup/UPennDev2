local K = {}
local T = require'libTransform'
local ffi = require'ffi'
local ptorch = require'util'.ptorch
local sin, cos = math.sin, math.cos
local asin, acos = math.asin, math.acos
local sqrt, atan2 = math.sqrt, math.atan2
local PI = math.pi
local shoulderOffsetX = 0;    
local shoulderOffsetY = 0.234;
local shoulderOffsetZ = 0.165;
local upperArmLength = .246;
local elbowOffsetX =   .030; 
--local lowerArmLength = .186; //Default 7DOF arm
local lowerArmLength = .250; -- LONGARM model
local dUpperArm = sqrt(upperArmLength*upperArmLength+elbowOffsetX*elbowOffsetX)
local dLowerArm = sqrt(lowerArmLength*lowerArmLength+elbowOffsetX*elbowOffsetX)
local aUpperArm = math.atan(elbowOffsetX/upperArmLength);
local aLowerArm = math.atan(elbowOffsetX/lowerArmLength);

-- trArm: Desired transform of the arm. Assume body rotations are removed
local function ik(trArm, qOrg, shoulderYaw, FLIP_SHOULDER_ROLL)

  -- TODO: Use the cdata directly, rather than in and out of C
  local xWrist = ffi.new('double[3]', T.get_pos(trArm))
  local dWrist = xWrist[0]^2+xWrist[1]^2+xWrist[2]^2
  local cElbow = .5*(dWrist-dUpperArm^2-dLowerArm^2)/(dUpperArm*dLowerArm);
  --print('cElbow', cElbow, 'dWrist', dWrist, dUpperArm^2+dLowerArm^2)
  if (cElbow > 1) then cElbow = 1 elseif (cElbow < -1) then cElbow = -1 end
  local elbowPitch = -acos(cElbow)-aUpperArm-aLowerArm;
  
  -- From shoulder yaw to wrist 
  local m = T.rotX(shoulderYaw)
    * T.trans(upperArmLength, 0, elbowOffsetX)
    * T.rotY(elbowPitch)
    * T.trans(lowerArmLength,0,-elbowOffsetX)
  
  --ptorch(m)
  
  local a = m[1][4]^2 + m[2][4]^2
  local b = -m[1][4]*xWrist[1];
  local c = xWrist[1]^2 - m[2][4]^2
  --
  local shoulderRoll
  -- NaN handling
  if ((b*b-a*c<0) or a==0 ) then
    shoulderRoll = 0
  else
    local c21,c22,s21,s22;
    s21= (-b+sqrt(b*b-a*c))/a;
    s22= (-b-sqrt(b*b-a*c))/a;
    if (s21 > 1) then s21 = 1; elseif (s21 < -1) then s21 = -1; end
    if (s22 > 1) then s22 = 1; elseif (s22 < -1) then s22 = -1; end
    local shoulderRoll1 = asin(s21);
    local shoulderRoll2 = asin(s22);
    local err1 = s21*m[1][4]+cos(shoulderRoll1)*m[2][4]-xWrist[1];
    local err2 = s22*m[1][4]+cos(shoulderRoll2)*m[2][4]-xWrist[1];
    if (err1*err1<err2*err2) then
      shoulderRoll = shoulderRoll1;
    else
      shoulderRoll = shoulderRoll2;
    end
  end
  if FLIP_SHOULDER_ROLL then
    if (arm==ARM_LEFT) then
      shoulderRoll = PI-shoulderRoll;
    else
      shoulderRoll = (-PI) - shoulderRoll;
    end
  end
  
  local s2 = sin(shoulderRoll);
  local c2 = cos(shoulderRoll);
  local t1 = -c2 * m[1][4] + s2 * m[2][4];
 
  local c1 = (m[3][4]*xWrist[2]-t1*xWrist[0]) /(m[3][4]*m[3][4] + t1^2);
  local s1 = (m[3][4]*xWrist[0]+t1*xWrist[2]) /(m[3][4]*m[3][4] + t1^2);

  local shoulderPitch = atan2(s1,c1);
  
  -- Now we know shoulder pich, roll, yaw and elbow pitch
  -- Calc the final transform for the wrist based on rotation alone

  local tRot = T.rotY(shoulderPitch) * T.rotZ(shoulderRoll) * T.rotX(shoulderYaw) * T.rotY(elbowPitch)
  local tInvRot = T.inv(tRot)
  local rotWrist = tInvRot * trArm;
  
  local wristYaw_a, wristRoll_a, wristYaw2_a, wristYaw_b, wristRoll_b, wristYaw2_b;
  -- Two solutions
  wristRoll_a = acos(rotWrist[1][1]); -- 0 to pi
  local swa = sin(wristRoll_a);
  wristYaw_a = atan2 (rotWrist[3][1]*swa,rotWrist[2][1]*swa);
  wristYaw2_a = atan2 (rotWrist[1][3]*swa,-rotWrist[1][2]*swa);

  -- Flipped position
  wristRoll_b = -wristRoll_a; -- -pi to 0
  local swb = sin(wristRoll_b);
  wristYaw_b = atan2 (rotWrist[3][1]*swb,rotWrist[2][1]*swb);  
  wristYaw2_b = atan2 (rotWrist[1][3]*swb,-rotWrist[1][2]*swb);
  -- singular point: just use current angles
  if(swa<0.00001) then
    wristYaw_a = qOrg[5];    
    wristRoll_a = qOrg[6];
    wristYaw2_a = qOrg[7];
    wristYaw_b = qOrg[5];
    wristRoll_b = qOrg[6];
    wristYaw2_b = qOrg[7];
  end
  
  local select_a = false;
  local err_a = ( (qOrg[5] - wristYaw_a+5*PI) % 2*PI ) - PI;
  local err_b = ( (qOrg[5] - wristYaw_b+5*PI) % 2*PI ) - PI;
  if (err_a*err_a<err_b*err_b) then select_a=true; end
  
  local qArm_t = torch.Tensor(7)
  local qArm_d = qArm_t:data()
  
  qArm_d[0] = shoulderPitch;
  qArm_d[1] = shoulderRoll;
  qArm_d[2] = shoulderYaw;
  qArm_d[3] = elbowPitch;

  if (select_a==true) then
    qArm_d[4] = wristYaw_a;
    qArm_d[5] = wristRoll_a;
    qArm_d[6] = wristYaw2_a;
  else
    qArm_d[4] = wristYaw_b;
    qArm_d[5] = wristRoll_b;
    qArm_d[6] = wristYaw2_b;
  end
  return qArm_t
end

K.ik = ik

return K