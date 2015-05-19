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
local fabs = math.abs
local min, max = require'math'.min, require'math'.max
local sqrt, atan2, atan = require'math'.sqrt, require'math'.atan2, require'math'.atan
local PI = require'math'.pi
local TWO_PI = 2 * PI
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

function M.com(qWaist, qLArm, qRArm, qLLeg, qRLeg)
	local com = vnew()



	return com
end

--[[
std::vector<double>
THOROP_kinematics_calculate_com_positions(
    const double *qWaist,
    const double *qLArm,
    const double *qRArm,
    const double *qLLeg,
    const double *qRLeg,

    double mLHand,
    double mRHand,
    double bodyPitch,

    int use_lleg,
    int use_rleg
    ){


  Transform
    tPelvis, tTorso,
    tLArm0, tLArm1, tLArm2, tLArm3, tLArm4,
    tRArm0, tRArm1, tRArm2, tRArm3, tRArm4,

    tLLeg0, tLLeg1, tLLeg2, tLLeg3, tLLeg4, tLLeg5,
    tRLeg0, tRLeg1, tRLeg2, tRLeg3, tRLeg4, tRLeg5,

    tPelvisCOM, tTorsoCOM
    ;


  tPelvis = tPelvis
    .rotateY(bodyPitch);

  tTorso = trcopy(tPelvis)
    .rotateZ(qWaist[0])
    .rotateY(qWaist[1]);

  tLArm0= trcopy(tTorso).translate(armLink[0])
          .rotateY(qLArm[0]);

  tLArm1= trcopy(tTorso).translate(armLink[0])
          .rotateY(qLArm[0]).rotateZ(qLArm[1])
          .translate(armCom[1]);

  tLArm2= trcopy(tTorso).translate(armLink[0])
          .rotateY(qLArm[0]).rotateZ(qLArm[1])
          .translate(armLink[2]).rotateX(qLArm[2])
          .translate(armCom[2]);

  tLArm3= trcopy(tTorso).translate(armLink[0])
          .rotateY(qLArm[0]).rotateZ(qLArm[1])
          .translate(armLink[2]).rotateX(qLArm[2])
          .translate(armLink[3]).rotateY(qLArm[3])
          .translate(armCom[3]);

  tLArm4= trcopy(tTorso).translate(armLink[0])
          .rotateY(qLArm[0]).rotateZ(qLArm[1])
          .translate(armLink[2]).rotateX(qLArm[2])
          .translate(armLink[3]).rotateY(qLArm[3])
          .translate(armLink[4]).rotateX(qLArm[4])
          .translate(armCom[4]);

  tRArm0= trcopy(tTorso).translate(rarmLink0)
          .rotateY(qRArm[0]);

  tRArm1= trcopy(tTorso).translate(rarmLink0)
          .rotateY(qRArm[0]).rotateZ(qRArm[1])
          .translate(armCom[1]);

  tRArm2= trcopy(tTorso).translate(rarmLink0)
          .rotateY(qRArm[0]).rotateZ(qRArm[1])
          .translate(armLink[2]).rotateX(qRArm[2])
          .translate(armCom[2]);

  tRArm3= trcopy(tTorso).translate(rarmLink0)
          .rotateY(qRArm[0]).rotateZ(qRArm[1])
          .translate(armLink[2]).rotateX(qRArm[2])
          .translate(armLink[3]).rotateY(qRArm[3])
          .translate(armCom[3]);

  tRArm4= trcopy(tTorso).translate(rarmLink0)
          .rotateY(qRArm[0]).rotateZ(qRArm[1])
          .translate(armLink[2]).rotateX(qRArm[2])
          .translate(armLink[3]).rotateY(qRArm[3])
          .translate(armLink[4]).rotateX(qRArm[4])
          .translate(armCom[4]);


  tLLeg0 = trcopy(tPelvis).translate(llegLink0)
          .rotateZ(qLLeg[0])
          .translate(legCom[0]);
  tLLeg1 = trcopy(tPelvis).translate(llegLink0)
          .rotateZ(qLLeg[0]).rotateX(qLLeg[1])
          .translate(legCom[1]);
  tLLeg2 = trcopy(tPelvis).translate(llegLink0)
          .rotateZ(qLLeg[0]).rotateX(qLLeg[1]).rotateY(qLLeg[2])
          .translate(legCom[2]);
  tLLeg3 = trcopy(tPelvis).translate(llegLink0)
          .rotateZ(qLLeg[0]).rotateX(qLLeg[1]).rotateY(qLLeg[2])
          .translate(legLink[3]).rotateY(qLLeg[3])
          .translate(legCom[3]);
  tLLeg4 = trcopy(tPelvis).translate(llegLink0)
          .rotateZ(qLLeg[0]).rotateX(qLLeg[1]).rotateY(qLLeg[2])
          .translate(legLink[3]).rotateY(qLLeg[3])
          .translate(legLink[4]).rotateY(qLLeg[4])
          .translate(legCom[4]);
  tLLeg5 = trcopy(tPelvis).translate(llegLink0)
          .rotateZ(qLLeg[0]).rotateX(qLLeg[1]).rotateY(qLLeg[2])
          .translate(legLink[3]).rotateY(qLLeg[3])
          .translate(legLink[4]).rotateY(qLLeg[4])
          .translate(legLink[5]).rotateY(qLLeg[5])
          .translate(legCom[5]);

  tRLeg0 = trcopy(tPelvis).translate(rlegLink0)
          .rotateZ(qRLeg[0])
          .translate(legCom[6]);
  tRLeg1 = trcopy(tPelvis).translate(rlegLink0)
          .rotateZ(qRLeg[0]).rotateX(qRLeg[1])
          .translate(legCom[7]);
  tRLeg2 = trcopy(tPelvis).translate(rlegLink0)
          .rotateZ(qRLeg[0]).rotateX(qRLeg[1]).rotateY(qRLeg[2])
          .translate(legCom[8]);
  tRLeg3 = trcopy(tPelvis).translate(rlegLink0)
          .rotateZ(qRLeg[0]).rotateX(qRLeg[1]).rotateY(qRLeg[2])
          .translate(legLink[3]).rotateY(qRLeg[3])
          .translate(legCom[9]);
  tRLeg4 = trcopy(tPelvis).translate(rlegLink0)
          .rotateZ(qRLeg[0]).rotateX(qRLeg[1]).rotateY(qRLeg[2])
          .translate(legLink[3]).rotateY(qRLeg[3])
          .translate(legLink[4]).rotateY(qRLeg[4])
          .translate(legCom[10]);
  tRLeg5 = trcopy(tPelvis).translate(rlegLink0)
          .rotateZ(qRLeg[0]).rotateX(qRLeg[1]).rotateY(qRLeg[2])
          .translate(legLink[3]).rotateY(qRLeg[3])
          .translate(legLink[4]).rotateY(qRLeg[4])
          .translate(legLink[5]).rotateY(qRLeg[5])
          .translate(legCom[11]);


  /////////////////////////////////

  tPelvisCOM = tPelvis
    .translateX(comPelvisX)
    .translateZ(comPelvisZ);

  tTorsoCOM = tTorso
    .translateX(comTorsoX)
    .translateZ(comTorsoZ);

//make a single compound COM position (from pelvis frame)
  std::vector<double> r(4);


 r[0] =
         mPelvis * tPelvisCOM(0,3) +
         mTorso * tTorsoCOM(0,3) +

         MassArm[0]* (tLArm0(0,3)+tRArm0(0,3))+
         MassArm[1]* (tLArm1(0,3)+tRArm1(0,3))+
         MassArm[2]* (tLArm2(0,3)+tRArm2(0,3))+
         MassArm[3]* (tLArm3(0,3)+tRArm3(0,3))+
         MassArm[4]* (tLArm4(0,3)+tRArm4(0,3))+

         MassLeg[0]* (tLLeg0(0,3)*use_lleg+tRLeg0(0,3)*use_rleg)+
         MassLeg[1]* (tLLeg1(0,3)*use_lleg+tRLeg1(0,3)*use_rleg)+
         MassLeg[2]* (tLLeg2(0,3)*use_lleg+tRLeg2(0,3)*use_rleg)+
         MassLeg[3]* (tLLeg3(0,3)*use_lleg+tRLeg3(0,3)*use_rleg)+
         MassLeg[4]* (tLLeg4(0,3)*use_lleg+tRLeg4(0,3)*use_rleg)+
         MassLeg[5]* (tLLeg5(0,3)*use_lleg+tRLeg5(0,3)*use_rleg);


  r[1] = mPelvis * tPelvisCOM(1,3) +
         mTorso * tTorsoCOM(1,3) +

         MassArm[0]* (tLArm0(1,3)+tRArm0(1,3))+
         MassArm[1]* (tLArm1(1,3)+tRArm1(1,3))+
         MassArm[2]* (tLArm2(1,3)+tRArm2(1,3))+
         MassArm[3]* (tLArm3(1,3)+tRArm3(1,3))+
         MassArm[4]* (tLArm4(1,3)+tRArm4(1,3))+

         MassLeg[0]* (tLLeg0(1,3)*use_lleg+tRLeg0(1,3)*use_rleg)+
         MassLeg[1]* (tLLeg1(1,3)*use_lleg+tRLeg1(1,3)*use_rleg)+
         MassLeg[2]* (tLLeg2(1,3)*use_lleg+tRLeg2(1,3)*use_rleg)+
         MassLeg[3]* (tLLeg3(1,3)*use_lleg+tRLeg3(1,3)*use_rleg)+
         MassLeg[4]* (tLLeg4(1,3)*use_lleg+tRLeg4(1,3)*use_rleg)+
         MassLeg[5]* (tLLeg5(1,3)*use_lleg+tRLeg5(1,3)*use_rleg);

  r[2] = mPelvis * tPelvisCOM(2,3) +
         mTorso * tTorsoCOM(2,3) +

         MassArm[0]* (tLArm0(2,3)+tRArm0(2,3))+
         MassArm[1]* (tLArm1(2,3)+tRArm1(2,3))+
         MassArm[2]* (tLArm2(2,3)+tRArm2(2,3))+
         MassArm[3]* (tLArm3(2,3)+tRArm3(2,3))+
         MassArm[4]* (tLArm4(2,3)+tRArm4(2,3))+

         MassLeg[0]* (tLLeg0(2,3)*use_lleg+tRLeg0(2,3)*use_rleg)+
         MassLeg[1]* (tLLeg1(2,3)*use_lleg+tRLeg1(2,3)*use_rleg)+
         MassLeg[2]* (tLLeg2(2,3)*use_lleg+tRLeg2(2,3)*use_rleg)+
         MassLeg[3]* (tLLeg3(2,3)*use_lleg+tRLeg3(2,3)*use_rleg)+
         MassLeg[4]* (tLLeg4(2,3)*use_lleg+tRLeg4(2,3)*use_rleg)+
         MassLeg[5]* (tLLeg5(2,3)*use_lleg+tRLeg5(2,3)*use_rleg);

  int i;

  r[3] = mPelvis + mTorso;

  for (i=0;i<7;i++) r[3]+=2*MassArm[i];
  for (i=0;i<6;i++) r[3]+=(use_lleg+use_rleg)*MassLeg[i];

  return r;
}
--]]
return K
