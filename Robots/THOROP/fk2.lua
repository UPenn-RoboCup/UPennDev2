local K = {}
local sin, cos = math.sin, math.cos
  local shoulderOffsetX = 0;    
  local shoulderOffsetY = 0.234;
  local shoulderOffsetZ = 0.165;
  local upperArmLength = .246;
  local elbowOffsetX =   .030; 
  --local lowerArmLength = .186; //Default 7DOF arm
  local lowerArmLength = .250; -- LONGARM model
  
local torch = require"torch"
local function fk(q)
local c1, s1 = cos(q[1]), sin(q[1])
local c2, s2 = cos(q[2]), sin(q[2])
local c3, s3 = cos(q[3]), sin(q[3])
local c4, s4 = cos(q[4]), sin(q[4])
local c5, s5 = cos(q[5]), sin(q[5])
local c6, s6 = cos(q[6]), sin(q[6])
local c7, s7 = cos(q[7]), sin(q[7])
return {{(((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*s5 + (s1*s3 - s2*c1*c3)*c5)*s6 + (-(s1*c3 + s2*s3*c1)*s4 + c1*c2*c4)*c6, ((((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*s5 + (s1*s3 - s2*c1*c3)*c5)*c6 - (-(s1*c3 + s2*s3*c1)*s4 + c1*c2*c4)*s6)*c7 + (((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*c5 - (s1*s3 - s2*c1*c3)*s5)*s7, -((((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*s5 + (s1*s3 - s2*c1*c3)*c5)*c6 - (-(s1*c3 + s2*s3*c1)*s4 + c1*c2*c4)*s6)*s7 + (((s1*c3 + s2*s3*c1)*c4 + s4*c1*c2)*c5 - (s1*s3 - s2*c1*c3)*s5)*c7, -0.25*(s1*c3 + s2*s3*c1)*s4 - 0.03*(s1*c3 + s2*s3*c1)*c4 + 0.03*s1*c3 + 0.03*s2*s3*c1 - 0.03*s4*c1*c2 + 0.25*c1*c2*c4 + 0.246*c1*c2}, {((s2*s4 - s3*c2*c4)*s5 + c2*c3*c5)*s6 + (s2*c4 + s3*s4*c2)*c6, (((s2*s4 - s3*c2*c4)*s5 + c2*c3*c5)*c6 - (s2*c4 + s3*s4*c2)*s6)*c7 + ((s2*s4 - s3*c2*c4)*c5 - s5*c2*c3)*s7, -(((s2*s4 - s3*c2*c4)*s5 + c2*c3*c5)*c6 - (s2*c4 + s3*s4*c2)*s6)*s7 + ((s2*s4 - s3*c2*c4)*c5 - s5*c2*c3)*c7, -0.03*s2*s4 + 0.25*s2*c4 + 0.246*s2 + 0.25*s3*s4*c2 + 0.03*s3*c2*c4 - 0.03*s3*c2}, {(((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*s5 + (s1*s2*c3 + s3*c1)*c5)*s6 + (-(-s1*s2*s3 + c1*c3)*s4 - s1*c2*c4)*c6, ((((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*s5 + (s1*s2*c3 + s3*c1)*c5)*c6 - (-(-s1*s2*s3 + c1*c3)*s4 - s1*c2*c4)*s6)*c7 + (((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*c5 - (s1*s2*c3 + s3*c1)*s5)*s7, -((((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*s5 + (s1*s2*c3 + s3*c1)*c5)*c6 - (-(-s1*s2*s3 + c1*c3)*s4 - s1*c2*c4)*s6)*s7 + (((-s1*s2*s3 + c1*c3)*c4 - s1*s4*c2)*c5 - (s1*s2*c3 + s3*c1)*s5)*c7, -0.25*(-s1*s2*s3 + c1*c3)*s4 - 0.03*(-s1*s2*s3 + c1*c3)*c4 - 0.03*s1*s2*s3 + 0.03*s1*s4*c2 - 0.25*s1*c2*c4 - 0.246*s1*c2 + 0.03*c1*c3}, {0, 0, 0, 1}}
end
local function fk_t(tr_d, q)
local c1, s1 = cos(q[1]), sin(q[1])
local c2, s2 = cos(q[2]), sin(q[2])
local c3, s3 = cos(q[3]), sin(q[3])
local c4, s4 = cos(q[4]), sin(q[4])
local c5, s5 = cos(q[5]), sin(q[5])
local c6, s6 = cos(q[6]), sin(q[6])
local c7, s7 = cos(q[7]), sin(q[7])
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
end
K.fk = fk
K.fk_t = fk_t
return K
