#include "THOROPKinematics.h"
//For THOR mk2



std::vector<double>THOROP_kinematics_forward_joints(const double *r){
  /* forward kinematics to convert servo positions to joint angles */
  std::vector<double> q(23);
  for (int i = 0; i < 23; i++) {
    q[i] = r[i];
  }
  return q;
}

//DH transform params: (alpha, a, theta, d)

Transform THOROP_kinematics_forward_head(const double *q){
  Transform t;
  t = t.translateZ(neckOffsetZ)
    .mDH(0, 0, q[0], 0)
    .mDH(-PI/2, 0, -PI/2+q[1], 0)
    .rotateX(PI/2).rotateY(PI/2);
  return t;
}

Transform THOROP_kinematics_forward_l_leg(const double *q){
  Transform t;
  t = t.translateY(hipOffsetY).translateZ(-hipOffsetZ)
    .mDH(0, 0, PI/2+q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, aThigh+q[2], 0)
    .mDH(0, -dThigh, -aThigh-aTibia+q[3], 0)
    .mDH(0, -dTibia, aTibia+q[4], 0)
    .mDH(-PI/2, 0, q[5], 0)
    .rotateZ(PI).rotateY(-PI/2).translateZ(-footHeight);
  return t;
}

Transform THOROP_kinematics_forward_r_leg(const double *q){
  Transform t;
  t = t.translateY(-hipOffsetY).translateZ(-hipOffsetZ)
    .mDH(0, 0, PI/2+q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, aThigh+q[2], 0)
    .mDH(0, -dThigh, -aThigh-aTibia+q[3], 0)
    .mDH(0, -dTibia, aTibia+q[4], 0)
    .mDH(-PI/2, 0, q[5], 0)
    .rotateZ(PI).rotateY(-PI/2).translateZ(-footHeight);
  return t;
}


//This does not have toe or heel automatic tilt - we no longer use this 
std::vector<double> THOROP_kinematics_inverse_leg(Transform trLeg, int leg, double aShiftX, double aShiftY){
  std::vector<double> qLeg(6);
  Transform trInvLeg = inv(trLeg);

  // Hip Offset vector in Torso frame
  double xHipOffset[3];
  xHipOffset[0] = 0;
  xHipOffset[2] = -hipOffsetZ;
  if (leg == LEG_LEFT) xHipOffset[1] = hipOffsetY;
  else xHipOffset[1] = -hipOffsetY;

  // Hip Offset in Leg frame
  double xLeg[3];
  for (int i = 0; i < 3; i++) xLeg[i] = xHipOffset[i];
  trInvLeg.apply(xLeg);

  // Knee pitch
  double dLeg = xLeg[0]*xLeg[0] + xLeg[1]*xLeg[1] + (xLeg[2]-footHeight)*(xLeg[2]-footHeight);
  double cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);
  
  xLeg[2] -= footHeight;
  
  if (cKnee > 1) cKnee = 1;
  if (cKnee < -1) cKnee = -1;
  double kneePitch = acos(cKnee);

  // Ankle pitch and roll
  double ankleRoll = atan2(xLeg[1], xLeg[2]);
  double lLeg = sqrt(dLeg);
  if (lLeg < 1e-16) lLeg = 1e-16;
  double pitch0 = asin(dThigh*sin(kneePitch)/lLeg);
  double anklePitch = asin(-xLeg[0]/lLeg) - pitch0;

  Transform rHipT = trLeg;
  rHipT = rHipT.rotateX(-ankleRoll).rotateY(-anklePitch-kneePitch);

  double hipYaw = atan2(-rHipT(0,1), rHipT(1,1));
  double hipRoll = asin(rHipT(2,1));
  double hipPitch = atan2(-rHipT(2,0), rHipT(2,2));

  // Need to compensate for KneeOffsetX:
  qLeg[0] = hipYaw;
  qLeg[1] = hipRoll;
  qLeg[2] = hipPitch-aThigh;
  qLeg[3] = kneePitch+aThigh+aTibia;
  qLeg[4] = anklePitch-aTibia;
  qLeg[5] = ankleRoll;
  return qLeg;
}

//new leg IK with automatic toe/hill lift

std::vector<double> THOROP_kinematics_inverse_leg_heellift(Transform trLeg, int leg, double aShiftX, double aShiftY, int birdwalk){

  std::vector<double> qLeg(6);
  Transform trInvLeg = inv(trLeg);

  // Hip Offset vector in Torso frame
  double xHipOffset[3];
  xHipOffset[0] = 0;
  xHipOffset[2] = -hipOffsetZ;
  if (leg == LEG_LEFT) xHipOffset[1] = hipOffsetY;
  else xHipOffset[1] = -hipOffsetY;

  // Hip Offset in Leg frame
  double xLeg[3];
  for (int i = 0; i < 3; i++) xLeg[i] = xHipOffset[i];
  trInvLeg.apply(xLeg);

  //primary axes for the ground frame
  double vecx0 = cos(aShiftY);
  double vecx1 = 0;
  double vecx2 = sin(aShiftY);
//  double vecy0 = sin(aShiftY)*cos(aShiftX);
//  double vecy1 = cos(aShiftX);
//  double vecy2 = cos(aShiftY)*sin(aShiftX);
  double vecz0 = sin(aShiftY)*cos(aShiftX);
  double vecz1 = -sin(aShiftX);
  double vecz2 = cos(aShiftY)*cos(aShiftX);

  //Relative ankle position in global frame (origin is the landing position)
  double dAnkle0 = footHeight*vecz0;
  double dAnkle1 = footHeight*vecz1;
  double dAnkle2 = footHeight*vecz2;

  //Find relative torso position from ankle position (in global frame)
  double xAnkle0 = xLeg[0] - dAnkle0;
  double xAnkle1 = xLeg[1] - dAnkle1;
  double xAnkle2 = xLeg[2] - dAnkle2;

  //Calculate the knee pitch
  double dLeg = xAnkle0*xAnkle0 + xAnkle1*xAnkle1 + xAnkle2*xAnkle2;
  double cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);
  double ankle_tilt_angle = 0;
  double dLegMax = dTibia + dThigh;
  double footC = sqrt(footHeight*footHeight + footToeX*footToeX);
  double afootA = asin(footHeight/footC);

  if (dLeg>dLegMax*dLegMax) {
    //now we lift heel by x radian

    //  new Ankle position in surface frame:
    //   (toeX,0,0) - Fc*(cos(x+c),0,-sin(x+c))
    // = (toeX-Fc*cos(x+c),  0,   Fc*sin(x+c))

    //new ankle position (ax,ay,az) in global frame:
    // {  vecx0 * (toeX-Fc*cos(x+c)) + vecz0* (Fc*sin(x+c)),
    //    vecx1 * (toeX-Fc*cos(x+c)) + vecz1* (Fc*sin(x+c)),
    //    vecx2 * (toeX-Fc*cos(x+c)) + vecz2* (Fc*sin(x+c)),
    // }

    // or 
    // {  (vecx0 * toeX)    - vecx0*Fc*cos(b)+ vecz0*Fc*sin(b),
    //    (vecx1 * toeX)    - vecx1*Fc*cos(b)+ vecz1*Fc*sin(b),
    //    (vecx2 * toeX)    - vecx2*Fc*cos(b)+ vecz2*Fc*sin(b),
    // }

  // Leg distant constraint    
  // (xLeg[0]-ax)^2 + xLeg[1]^2 + (xLeg[2]-az)^2 = dLegMax^2

  // xLeg0Mod, yLeg0Mod, zLeg0Mod = xLeg[0]-vecx0*toeX, xLeg[1]-vecx1*toeX,xLeg[2]-vecx2*toeX
  // or 
  //  (xLeg0Mod + vecx0*Fc*cos(b) - vecz0*Fc*sin(b))^2 + 
  //  (xLeg1Mod + vecx1*Fc*cos(b) - vecz1*Fc*sin(b))^2 + 
  //  (xLeg2Mod + vecx2*Fc*cos(b) - vecz2*Fc*sin(b))^2 = dLegMax^2 
   
  // = (xLeg0Mod^2+xLeg1Mod^2+xLeg2Mod^2) + Fc^2 (vecx0^2+vecx1^2+vecx2^2) +
  //   2*Fc*cos(b) * (  xLeg0Mod*vecx0 + xLeg1Mod*vecx1 + xLeg2Mod*vecx2 ) +
  //   - 2*Fc*sin(b) * (  xLeg0Mod*vecz0 + xLeg1Mod*vecz1 + xLeg2Mod*vecz2 ) +    
  //   2*Fc*Fc*cos(b)sin(b)* (vecx0*vecz0 + vecx1*vecz1+ vecx2*vecz2)

  // eq: p*sinb + q*cosb + r* sinbcosb + s = 0

  double xLM0 = xLeg[0]-vecx0*footToeX;
  double xLM1 = xLeg[1]-vecx1*footToeX;
  double xLM2 = xLeg[2]-vecx2*footToeX;

  double s2 = (xLM0*xLM0+xLM1*xLM1+xLM2*xLM2) + footC*footC*(vecx0*vecx0+vecx1*vecx1+vecx2*vecx2)- dLegMax*dLegMax;
  double p2 = -2*footC* (xLM0*vecz0 + xLM1*vecz1 + xLM2*vecz2);
  double q2 = 2*footC* (xLM0*vecx0 + xLM1*vecx1 + xLM2*vecx2);
  double r2 = 2*footC*footC*(vecx0*vecz0 + vecx1*vecz1+vecx2*vecz2);

//newton method to find the solution
  double x0 = 0;
  double ferr=0;
  int iter_count=0;
  bool not_done=true; 
  while ((iter_count++<10) && not_done){
    ferr = p2*sin(x0)+q2*cos(x0)+r2*sin(x0)*cos(x0)+s2;
    double fdot = p2*cos(x0) - q2*sin(x0) + r2*cos(x0)*cos(x0) - r2*sin(x0)*sin(x0);
    x0 = x0 - ferr/fdot;
    if (fabs(ferr)<0.001) not_done=false;
  }  
  if (fabs(ferr)<0.01){ 
    ankle_tilt_angle = x0-afootA;
  }
  else{
    ankle_tilt_angle = 0;
  }
    
//    if (ankle_tilt_angle>45*3.1415/180)  ankle_tilt_angle=45*3.1415/180;
  
    //TODO: ankle location incorporiating surface angles
    xLeg[0] = xLeg[0] - (footToeX - footC*cos(ankle_tilt_angle+afootA));
    xLeg[2] = xLeg[2] - footC*sin(ankle_tilt_angle+afootA);
    dLeg = xLeg[0]*xLeg[0] + xLeg[1]*xLeg[1] + xLeg[2]*xLeg[2];
    cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);
  }else{    
    xLeg[2]-= footHeight;
  }

  if (cKnee > 1) cKnee = 1;
  if (cKnee < -1) cKnee = -1;
  double kneePitch = acos(cKnee);
  double kneeOffsetA=1;
  if (birdwalk>0) {
    kneePitch=-kneePitch;
    kneeOffsetA=-1;
  }

  // Ankle pitch and roll
  double ankleRoll = atan2(xLeg[1], xLeg[2]);
  double lLeg = sqrt(dLeg);
  if (lLeg < 1e-16) lLeg = 1e-16;
  double pitch0 = asin(dThigh*sin(kneePitch)/lLeg);
  double anklePitch = asin(-xLeg[0]/lLeg) - pitch0;

  Transform rHipT = trLeg;

  rHipT = rHipT.rotateX(-ankleRoll).rotateY(-anklePitch-kneePitch);

  double hipYaw = atan2(-rHipT(0,1), rHipT(1,1));
  double hipRoll = asin(rHipT(2,1));
  double hipPitch = atan2(-rHipT(2,0), rHipT(2,2));

  // Need to compensate for KneeOffsetX:
  qLeg[0] = hipYaw;
  qLeg[1] = hipRoll;
  qLeg[2] = hipPitch-aThigh*kneeOffsetA;
  qLeg[3] = kneePitch+aThigh*kneeOffsetA+aTibia*kneeOffsetA;
  qLeg[4] = anklePitch-aTibia*kneeOffsetA;
  qLeg[5] = ankleRoll;

  qLeg[4] = qLeg[4]+ankle_tilt_angle;
  return qLeg;
}


std::vector<double> THOROP_kinematics_inverse_leg_toelift(Transform trLeg, int leg, double aShiftX, double aShiftY,int birdwalk){

  //TODOTODOTODOTODOTODO!!!!!!!!!!!!!!
  std::vector<double> qLeg(6);
  Transform trInvLeg = inv(trLeg);

  // Hip Offset vector in Torso frame
  double xHipOffset[3];
  xHipOffset[0] = 0;
  xHipOffset[2] = -hipOffsetZ;
  if (leg == LEG_LEFT) xHipOffset[1] = hipOffsetY;
  else xHipOffset[1] = -hipOffsetY;

  // Hip Offset in Leg frame
  double xLeg[3];
  for (int i = 0; i < 3; i++) xLeg[i] = xHipOffset[i];
  trInvLeg.apply(xLeg);

//primary axes for the ground frame
  double vecx0 = cos(aShiftY);
  double vecx1 = 0;
  double vecx2 = sin(aShiftY);
//  double vecy0 = sin(aShiftY)*cos(aShiftX);
//  double vecy1 = cos(aShiftX);
//  double vecy2 = cos(aShiftY)*sin(aShiftX);
  double vecz0 = sin(aShiftY)*cos(aShiftX);
  double vecz1 = -sin(aShiftX);
  double vecz2 = cos(aShiftY)*cos(aShiftX);

  //Relative ankle position in global frame (origin is the landing position)
  double dAnkle0 = footHeight*vecz0;
  double dAnkle1 = footHeight*vecz1;
  double dAnkle2 = footHeight*vecz2;

  //Find relative torso position from ankle position (in global frame)
  double xAnkle0 = xLeg[0] - dAnkle0;
  double xAnkle1 = xLeg[1] - dAnkle1;
  double xAnkle2 = xLeg[2] - dAnkle2;

  // Knee pitch
  double dLeg = xAnkle0*xAnkle0 + xAnkle1*xAnkle1 + xAnkle2*xAnkle2;
  double cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);
  double ankle_tilt_angle = 0;
  double dLegMax = dTibia + dThigh;
  double footC = sqrt(footHeight*footHeight + footHeelX*footHeelX);
  double afootA = asin(footHeight/footC);

  if (dLeg>dLegMax*dLegMax) {

    //now we lift toe by x radian 
    //  new Ankle position in surface frame:
    //   (-heelX,0,0) + Fc*(cos(x+c),0,sin(x+c))
    // = (-heelX + Fc*cos(x+c),  0,   Fc*sin(x+c))

    //new ankle position (ax,ay,az) in global frame:
    // {  vecx0 * (-heelX + Fc*cos(x+c)) + vecz0* (Fc*sin(x+c)),
    //    vecx1 * (-heelX + Fc*cos(x+c)) + vecz1* (Fc*sin(x+c)),
    //    vecx2 * (-heelX + Fc*cos(x+c)) + vecz2* (Fc*sin(x+c)),
    // }

    // or 
    // {  -(vecx0 * heelX)  + vecx0*Fc*cos(b)+ vecz0*Fc*sin(b),
    //    -(vecx1 * heelX)  + vecx1*Fc*cos(b)+ vecz1*Fc*sin(b),
    //    -(vecx2 * heelX)  + vecx2*Fc*cos(b)+ vecz2*Fc*sin(b),
    // }

  // Leg distant constraint    
  // (xLeg[0]-ax)^2 + xLeg[1]^2 + (xLeg[2]-az)^2 = dLegMax^2

  // xLeg0Mod, yLeg0Mod, zLeg0Mod = xLeg[0]+vecx0*heelX, xLeg[1]+vecx1*heelX,xLeg[2]+vecx2*heelX
  // or 
  //  (xLeg0Mod - vecx0*Fc*cos(b) - vecz0*Fc*sin(b))^2 + 
  //  (xLeg1Mod - vecx1*Fc*cos(b) - vecz1*Fc*sin(b))^2 + 
  //  (xLeg2Mod - vecx2*Fc*cos(b) - vecz2*Fc*sin(b))^2 = dLegMax^2 
   
  // = (xLeg0Mod^2+xLeg1Mod^2+xLeg2Mod^2) + Fc^2 (vecx0^2+vecx1^2+vecx2^2) +
  //   - 2*Fc*cos(b) * (  xLeg0Mod*vecx0 + xLeg1Mod*vecx1 + xLeg2Mod*vecx2 ) +
  //   - 2*Fc*sin(b) * (  xLeg0Mod*vecz0 + xLeg1Mod*vecz1 + xLeg2Mod*vecz2 ) +    
  //   2*Fc*Fc*cos(b)sin(b)* (vecx0*vecz0 + vecx1*vecz1+ vecx2*vecz2)

//   2*Fc*Fc*cos(b)sin(b)* (vecx0*vecz0 + vecx1*vecz1+ vecx2*vecz2)

  // eq: p*sinb + q*cosb + r* sinbcosb + s = 0

  double xLM0 = xLeg[0]+vecx0*footHeelX;
  double xLM1 = xLeg[1]+vecx1*footHeelX;
  double xLM2 = xLeg[2]+vecx2*footHeelX;

  double s2 = (xLM0*xLM0+xLM1*xLM1+xLM2*xLM2) + footC*footC*(vecx0*vecx0+vecx1*vecx1+vecx2*vecx2)- dLegMax*dLegMax;
  double p2 = -2*footC* (xLM0*vecz0 + xLM1*vecz1 + xLM2*vecz2);
  double q2 = -2*footC* (xLM0*vecx0 + xLM1*vecx1 + xLM2*vecx2);
  double r2 = 2*footC*footC*(vecx0*vecz0 + vecx1*vecz1+vecx2*vecz2);

//newton method to find the solution
  double x0 = 0;
  double ferr=0;
  int iter_count=0;
  bool not_done=true; 
  while ((iter_count++<10) && not_done){
    ferr = p2*sin(x0)+q2*cos(x0)+r2*sin(x0)*cos(x0)+s2;
    double fdot = p2*cos(x0) - q2*sin(x0) + r2*cos(x0)*cos(x0) - r2*sin(x0)*sin(x0);
    x0 = x0 - ferr/fdot;
    if (fabs(ferr)<0.001) not_done=false;
  }  
  if (fabs(ferr)<0.01){ 
    ankle_tilt_angle = x0-afootA;
  }
  else{
    ankle_tilt_angle = 0;
  }

//    if (ankle_tilt_angle<-45*3.1415/180)  ankle_tilt_angle=-45*3.1415/180;

  //TODO: ankle location incorporiating surface angles 
  xLeg[0] = xLeg[0] + footHeelX - footC*cos(-ankle_tilt_angle+afootA);
  xLeg[2] = xLeg[2] - sin(afootA-ankle_tilt_angle)*footC;

  ankle_tilt_angle = -ankle_tilt_angle; //change into ankle PITCH bias angle
    dLeg = xLeg[0]*xLeg[0] + xLeg[1]*xLeg[1] + xLeg[2]*xLeg[2];
    cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);
  }else{    

    xLeg[2] -= footHeight;
  }


  if (cKnee > 1) cKnee = 1;
  if (cKnee < -1) cKnee = -1;
  double kneePitch = acos(cKnee);
  double kneeOffsetA=1;
  if (birdwalk>0) {
    kneePitch=-kneePitch;
    kneeOffsetA=-1;
  }

  // Ankle pitch and roll
  double ankleRoll = atan2(xLeg[1], xLeg[2]);
  double lLeg = sqrt(dLeg);
  if (lLeg < 1e-16) lLeg = 1e-16;
  double pitch0 = asin(dThigh*sin(kneePitch)/lLeg);
  double anklePitch = asin(-xLeg[0]/lLeg) - pitch0;

  Transform rHipT = trLeg;

  rHipT = rHipT.rotateX(-ankleRoll).rotateY(-anklePitch-kneePitch);

  double hipYaw = atan2(-rHipT(0,1), rHipT(1,1));
  double hipRoll = asin(rHipT(2,1));
  double hipPitch = atan2(-rHipT(2,0), rHipT(2,2));

  // Need to compensate for KneeOffsetX:
  qLeg[0] = hipYaw;
  qLeg[1] = hipRoll;
  qLeg[2] = hipPitch-aThigh*kneeOffsetA;
  qLeg[3] = kneePitch+aThigh*kneeOffsetA+aTibia*kneeOffsetA;
  qLeg[4] = anklePitch-aTibia*kneeOffsetA;
  qLeg[5] = ankleRoll;

  qLeg[4] = qLeg[4]+ankle_tilt_angle;
  return qLeg;
}


std::vector<double>THOROP_kinematics_inverse_l_leg(Transform trLeg, double aShiftX, double aShiftY){
  return THOROP_kinematics_inverse_leg(trLeg, LEG_LEFT,aShiftX,  aShiftY);}

std::vector<double>THOROP_kinematics_inverse_r_leg(Transform trLeg, double aShiftX, double aShiftY){
  return THOROP_kinematics_inverse_leg(trLeg, LEG_RIGHT, aShiftX,  aShiftY);}