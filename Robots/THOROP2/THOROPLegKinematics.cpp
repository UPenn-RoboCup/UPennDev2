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





/////////////////////////
//New correct leg IK code (that does NOT assume that foot is flat)
/////////////////////////




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

///////////////////////TODOTODOTODO
//TODO: ankle location incorporiating surface angles
//Now let's just assume flat surface

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
  double kneePitchActual = kneePitch+aThigh*kneeOffsetA+aTibia*kneeOffsetA;





  //Now we know knee pitch and ankle tilt 
  Transform trAnkle =  trcopy (trLeg);

  //Body to Leg FK:
  //Trans(HIP).Rz(q0).Rx(q1).Ry(q2).Trans(UL).Ry(q3).Trans(LL).Ry(q4).Rx(q5).Trans(Foot)
  
  //Genertae body to ankle transform 
  trAnkle = trAnkle
    .translate(footToeX,0,0)
    .rotateY(ankle_tilt_angle)
    .translate(-footToeX,0,footHeight);

  //Get rid of hip offset to make hip to ankle transform
  Transform t;
  if (leg == LEG_LEFT){
    t=t.translate(0,-hipOffsetY,hipOffsetZ)*trAnkle;      
  }else{
    t=t.translate(0,hipOffsetY,hipOffsetZ)*trAnkle;
  }
  //then now t = Rz(q0).Rx(q1).Ry(q2).Trans(UL).Ry(q3).Trans(LL).Ry(q4).Rx(q5)
  //or     t_inv = Rx(-q5).Ry(-q4).   Trans(-LL).Ry(-q3).Trans(-UL)     .Ry(-q2).Rx(-q1).Rz(-q0)
  //       t_inv*(0 0 0 1)T  = Rx(-q5).Ry(-q4) * m * (0 0 0 1)T  
  Transform m;
  Transform tInv = inv(t);
  m=m.translate(kneeOffsetX,0,thighLength).rotateY(-kneePitchActual).translate(-kneeOffsetX,0,tibiaLength);

  //now lets solve for ankle pitch (q4) first
  // tInv(0,3) = m(0,3)*cos(q4) - m(2,3)*sin(q4)
  // (m0^2+m2^2)s^2 + 2t0 m2 s + t0^2 - m0^2 = 0

  double a = m(0,3)*m(0,3) + m(2,3)*m(2,3);
  double b = tInv(0,3) * m(2,3);
  double c = tInv(0,3)*tInv(0,3) - m(0,3)*m(0,3);
  double k = b*b-a*c;
  if (k<0) k=0;
  double s1 = (-b-sqrt(k))/a;
  double s2 = (-b+sqrt(k))/a;
  if (s1<-1) s1=-1;
  if (s2<-1) s2=-1;
  if (s1>1) s1=1;
  if (s2>1) s2=1;
  double anklePitch1 = asin(s1);
  double anklePitch2 = asin(s2);
  double err1 = tInv(0,3)-m(0,3)*cos(anklePitch1)+m(2,3)*sin(anklePitch1);
  double err2 = tInv(0,3)-m(0,3)*cos(anklePitch2)+m(2,3)*sin(anklePitch2);
  double anklePitchNew = anklePitch1;
  if (err1*err1>err2*err2) anklePitchNew = anklePitch2;


  //then now solve for ankle roll (q5)
  //tInv(1,3) = m0*sin(q4)* sin(q5) + m1*cos(q5) + m2*cos(q4)*sin(q5)
  //sin(q5) * (m0*sin(q4) + m2*cos(q4))  + m1*cos(q5) - tInv(1,3) = 0

  double p1 = m(0,3)*sin(anklePitchNew) + m(2,3)*cos(anklePitchNew);
  a = p1*p1+m(1,3)*m(1,3);
  b = -tInv(1,3)*p1;
  c = tInv(1,3)*tInv(1,3)-m(1,3)*m(1,3);
  k=b*b-a*c;
  if (k<0) k=0;
  s1 = (-b-sqrt(k))/a;
  s2 = (-b+sqrt(k))/a;
  if (s1<-1) s1=-1;
  if (s2<-1) s2=-1;
  if (s1>1) s1=1;
  if (s2>1) s2=1;
  double ankleRoll1 = asin(s1);
  double ankleRoll2 = asin(s2);
  err1 = tInv(1,3)-m(2,3)*cos(ankleRoll1)-p1*sin(ankleRoll1);
  err2 = tInv(1,3)-m(2,3)*cos(ankleRoll2)-p1*sin(ankleRoll2);
  double ankleRollNew = ankleRoll1;
  if (err1*err1>err2*err2) ankleRollNew = ankleRoll2;


  //Now we have ankle roll and pitch
  //again, t = Rz(q0).Rx(q1).Ry(q2).Trans(UL).Ry(q3).Trans(LL).Ry(q4).Rx(q5)
  //lets get rid of knee and ankle rotations
  t=t.rotateX(-ankleRollNew).rotateY(-anklePitchNew-kneePitchActual);

  //now use ZXY euler angle equation to get q0,q1,q2

  double hipRollNew = asin(t(2,1));
  double hipYawNew = atan2(-t(0,1),t(1,1));
  double hipPitchNew = atan2(-t(2,0),t(2,2));






  // Ankle pitch and roll
  double ankleRoll = atan2(xLeg[1], xLeg[2]);
  double lLeg = sqrt(dLeg);
  if (lLeg < 1e-16) lLeg = 1e-16;
  double pitch0 = asin(dThigh*sin(kneePitch)/lLeg);
  double anklePitch = asin(-xLeg[0]/lLeg) - pitch0;

  Transform rHipT = trcopy(trLeg);

  rHipT = rHipT.rotateX(-ankleRoll).rotateY(-anklePitch-kneePitch);

  double hipYaw = atan2(-rHipT(0,1), rHipT(1,1));
  double hipRoll = asin(rHipT(2,1));
  double hipPitch = atan2(-rHipT(2,0), rHipT(2,2));

  
  

  // Need to compensate for KneeOffsetX:
  qLeg[0] = hipYaw;
  qLeg[1] = hipRoll;
  qLeg[2] = hipPitch-aThigh*kneeOffsetA;
  qLeg[3] = kneePitchActual;
  qLeg[4] = anklePitch-aTibia*kneeOffsetA;
  qLeg[5] = ankleRoll;
  qLeg[4] = qLeg[4]+ankle_tilt_angle;


//TODO: pitch sometimes screws up

  if (hipPitchNew<-1.57*1.2) {
   printf("Pitch:%f\n",hipPitchNew*180/3.1415);
   printf("val: %f %f\n",t(2,0),t(2,2));
   hipPitchNew = qLeg[2];
  }

  if (hipPitchNew>1.57*1.2) {
   printf("Pitch:%f\n",hipPitchNew*180/3.1415);
   printf("val: %f %f\n",t(2,0),t(2,2));
   hipPitchNew = qLeg[2];
  }


//new IK
  qLeg[0] = hipYawNew;
  qLeg[1] = hipRollNew;
  qLeg[2] = hipPitchNew;
  qLeg[3] = kneePitchActual;
  qLeg[4] = anklePitchNew;
  qLeg[5] = ankleRollNew;


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
  double kneePitchActual = kneePitch+aThigh*kneeOffsetA+aTibia*kneeOffsetA;



  //Now we know knee pitch and ankle tilt 
  Transform trAnkle =  trcopy (trLeg);

  //Body to Leg FK:
  //Trans(HIP).Rz(q0).Rx(q1).Ry(q2).Trans(UL).Ry(q3).Trans(LL).Ry(q4).Rx(q5).Trans(Foot)
  
  //Genertae body to ankle transform 
  trAnkle = trAnkle
    .translate(footToeX,0,0)
    .rotateY(ankle_tilt_angle)
    .translate(-footToeX,0,footHeight);

  //Get rid of hip offset to make hip to ankle transform
  Transform t;
  if (leg == LEG_LEFT){
    t=t.translate(0,-hipOffsetY,hipOffsetZ)*trAnkle;      
  }else{
    t=t.translate(0,hipOffsetY,hipOffsetZ)*trAnkle;
  }
  //then now t = Rz(q0).Rx(q1).Ry(q2).Trans(UL).Ry(q3).Trans(LL).Ry(q4).Rx(q5)
  //or     t_inv = Rx(-q5).Ry(-q4).   Trans(-LL).Ry(-q3).Trans(-UL)     .Ry(-q2).Rx(-q1).Rz(-q0)
  //       t_inv*(0 0 0 1)T  = Rx(-q5).Ry(-q4) * m * (0 0 0 1)T  
  Transform m;
  Transform tInv = inv(t);
  m=m.translate(kneeOffsetX,0,thighLength).rotateY(-kneePitchActual).translate(-kneeOffsetX,0,tibiaLength);

  //now lets solve for ankle pitch (q4) first
  // tInv(0,3) = m(0,3)*cos(q4) - m(2,3)*sin(q4)
  // (m0^2+m2^2)s^2 + 2t0 m2 s + t0^2 - m0^2 = 0

  double a = m(0,3)*m(0,3) + m(2,3)*m(2,3);
  double b = tInv(0,3) * m(2,3);
  double c = tInv(0,3)*tInv(0,3) - m(0,3)*m(0,3);
  double k = b*b-a*c;
  if (k<0) k=0;
  double s1 = (-b-sqrt(k))/a;
  double s2 = (-b+sqrt(k))/a;
  if (s1<-1) s1=-1;
  if (s2<-1) s2=-1;
  if (s1>1) s1=1;
  if (s2>1) s2=1;
  double anklePitch1 = asin(s1);
  double anklePitch2 = asin(s2);
  double err1 = tInv(0,3)-m(0,3)*cos(anklePitch1)+m(2,3)*sin(anklePitch1);
  double err2 = tInv(0,3)-m(0,3)*cos(anklePitch2)+m(2,3)*sin(anklePitch2);
  double anklePitchNew = anklePitch1;
  if (err1*err1>err2*err2) anklePitchNew = anklePitch2;


  //then now solve for ankle roll (q5)
  //tInv(1,3) = m0*sin(q4)* sin(q5) + m1*cos(q5) + m2*cos(q4)*sin(q5)
  //sin(q5) * (m0*sin(q4) + m2*cos(q4))  + m1*cos(q5) - tInv(1,3) = 0

  double p1 = m(0,3)*sin(anklePitchNew) + m(2,3)*cos(anklePitchNew);
  a = p1*p1+m(1,3)*m(1,3);
  b = -tInv(1,3)*p1;
  c = tInv(1,3)*tInv(1,3)-m(1,3)*m(1,3);
  k=b*b-a*c;
  if (k<0) k=0;
  s1 = (-b-sqrt(k))/a;
  s2 = (-b+sqrt(k))/a;
  if (s1<-1) s1=-1;
  if (s2<-1) s2=-1;
  if (s1>1) s1=1;
  if (s2>1) s2=1;
  double ankleRoll1 = asin(s1);
  double ankleRoll2 = asin(s2);
  err1 = tInv(1,3)-m(2,3)*cos(ankleRoll1)-p1*sin(ankleRoll1);
  err2 = tInv(1,3)-m(2,3)*cos(ankleRoll2)-p1*sin(ankleRoll2);
  double ankleRollNew = ankleRoll1;
  if (err1*err1>err2*err2) ankleRollNew = ankleRoll2;


  //Now we have ankle roll and pitch
  //again, t = Rz(q0).Rx(q1).Ry(q2).Trans(UL).Ry(q3).Trans(LL).Ry(q4).Rx(q5)
  //lets get rid of knee and ankle rotations
  t=t.rotateX(-ankleRollNew).rotateY(-anklePitchNew-kneePitchActual);

  //now use ZXY euler angle equation to get q0,q1,q2

  double hipRollNew = asin(t(2,1));
  double hipYawNew = atan2(-t(0,1),t(1,1));
  double hipPitchNew = atan2(-t(2,0),t(2,2));













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




//TODO: pitch sometimes screws up

  if (hipPitchNew<-1.57*1.2) {
   printf("Pitch:%f\n",hipPitchNew*180/3.1415);
   printf("val: %f %f\n",t(2,0),t(2,2));
   hipPitchNew = qLeg[2];
  }

  if (hipPitchNew>1.57*1.2) {
   printf("Pitch:%f\n",hipPitchNew*180/3.1415);
   printf("val: %f %f\n",t(2,0),t(2,2));
   hipPitchNew = qLeg[2];
  }


//new IK
  qLeg[0] = hipYawNew;
  qLeg[1] = hipRollNew;
  qLeg[2] = hipPitchNew;
  qLeg[3] = kneePitchActual;
  qLeg[4] = anklePitchNew;
  qLeg[5] = ankleRollNew;


  
  return qLeg;
}


double THOROP_kinematics_inverse_leg_bodyheight_diff(
  Transform trLeg, int leg, double aShiftX, double aShiftY){
  
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

  double dLeg = xAnkle0*xAnkle0 + xAnkle1*xAnkle1 + xAnkle2*xAnkle2;
  double dLegMax = dTibia + dThigh;

  if (dLeg<dLegMax*dLegMax) {return 0.0;}

  //how much should we lower bodyheight? 
  // xAnkle0^2 + xAnkle1^2 + (xAnkle2-dh)^2 = dLeg^2
  double dh = xAnkle2 - sqrt(dLegMax*dLegMax - xAnkle0*xAnkle0 - xAnkle1*xAnkle1);
  return dh;
}





std::vector<double>THOROP_kinematics_inverse_l_leg(Transform trLeg, double aShiftX, double aShiftY){
  return THOROP_kinematics_inverse_leg(trLeg, LEG_LEFT,aShiftX,  aShiftY);}

std::vector<double>THOROP_kinematics_inverse_r_leg(Transform trLeg, double aShiftX, double aShiftY){
  return THOROP_kinematics_inverse_leg(trLeg, LEG_RIGHT, aShiftX,  aShiftY);}