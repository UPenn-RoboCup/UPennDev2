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

  //Find relative torso position from ankle position
  double xAnkle0 = xLeg[0] - footHeight*sin(aShiftY);
  double xAnkle1 = xLeg[1] - footHeight*sin(aShiftX);
  double xAnkle2 = xLeg[2] - footHeight*cos(aShiftY)*cos(aShiftX);

  // Knee pitch
  double dLeg = xAnkle0*xAnkle0 + xAnkle1*xAnkle1 + xAnkle2*xAnkle2;
  double cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);
  
  //Automatic heel lift when IK limit is reached
  double ankle_tilt_angle = 0;
  double dLegMax = dTibia + dThigh;
  double footC = sqrt(footHeight*footHeight + footToeX*footToeX);
  double afootA = asin(footHeight/footC);

  if (dLeg>dLegMax*dLegMax) {

    







    //Calculate the amount of heel lift
    // ax = footToeX-footC*cos(a+x), az = footC*sin(a+x)-footHeight
    // (xLeg[0]-ax)^2 + xLeg[1]^2 + (xLeg[2]-footHeight-az)^2 = dLegMax^2
    // or (xLeg0Mod + cosb*footC)^2 + xLeg[1]^2 + (xLeg[2]-sinb*footC)^2 = dLegMax^2
    //this eq: p * sinb + q*cosb + r = 0

    double xLegM0 = xAnkle0 - footToeX;
    double xLegM1 = xAnkle1;
    double xLegM2 = xAnkle2;

    double p = -2*footC*xLegM2;
    double q =  2*footC*xLegM0;
    double r = xLegM0*xLegM0 + xLegM1*xLegM1 + xLegM2*xLegM2 + footC*footC - dLegMax*dLegMax;

    double a = (p*p/q/q + 1);
    double b = 2*p*r/q/q;
    double c = r*r/q/q - 1; 
    double d = b*b-4*a*c;


//With base plane with aShiftY pitchangle
//The rotated ankle position is 
//ax:  footToeX * cos(aShiftY) - cos(a + aFootA + aShiftY)*footC
//ay:  sin(a+aFootA+aShiftY) * footC
  
    if (d > 0){
      double a1 = (-b + sqrt(d))/2/a;
      double a2 = (-b - sqrt(d))/2/a;
      double err1 = fabs(p*a1 + q*sqrt(1-a1*a1)+r);
      double err2 = fabs(p*a2 + q*sqrt(1-a2*a2)+r);
      double ankle_tilt_angle1 = asin(a1)-afootA-aShiftY;
      double ankle_tilt_angle2 = asin(a2)-afootA-aShiftY;
      if ((err1<0.0001) && (err2<0.0001)) { //we have two solutions
//        printf("Two lift angle: %.2f %.2f\n",-ankle_tilt_angle1*180/3.1415,-ankle_tilt_angle2*180/3.1415);
        if (fabs(ankle_tilt_angle1)<fabs(ankle_tilt_angle2))
          ankle_tilt_angle = ankle_tilt_angle1;
        else
          ankle_tilt_angle = ankle_tilt_angle2;
      }else{
        if (err1<err2) ankle_tilt_angle = ankle_tilt_angle1;
        else ankle_tilt_angle = ankle_tilt_angle2;
      }
  }else {
      ankle_tilt_angle = 0;
  }
  
//    if (ankle_tilt_angle>45*3.1415/180)  ankle_tilt_angle=45*3.1415/180;

    xLeg[0] = xLeg[0] - footToeX +footC*cos(ankle_tilt_angle+afootA);
    xLeg[2] = xLeg[2] - sin(afootA+ankle_tilt_angle)*footC;

/*  
    //Compensate the ankle position according to ankle tilt angle
    xLeg[0] = xLeg[0] + footToeX*cos(aShiftY) - 
              sin(aShiftY+ankle_tilt_angle)*footHeight - footToeX*cos(aShiftY+ankle_tilt_angle);
    xLeg[2] = xLeg[2] + footToeX*sin(aShiftY)
    - sin(afootA+ankle_tilt_angle+aShiftY)*footC;
*/


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

  //Find relative torso position from ankle position
  double xAnkle0 = xLeg[0] - footHeight*sin(aShiftY);
  double xAnkle1 = xLeg[1] - footHeight*sin(aShiftX);
  double xAnkle2 = xLeg[2] - footHeight*cos(aShiftY)*cos(aShiftX);

  // Knee pitch
  double dLeg = xAnkle0*xAnkle0 + xAnkle1*xAnkle1 + xAnkle2*xAnkle2;
  double cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);


  //Automatic toe lift when IK limit is reached
  double ankle_tilt_angle = 0;
  double dLegMax = dTibia + dThigh;
  double footC = sqrt(footHeight*footHeight + footHeelX*footHeelX);
  double afootA = asin(footHeight/footC);

  if (dLeg>dLegMax*dLegMax) {
    //with inclined surface
    //ORG ankle position : 
    //  footHeight*sin(aShiftY), footHeight*sin(aShiftX), footHeight*cos(aShiftY) * cos(aShiftX)
    //tilted ankle position, where b = theta + aFootA - aShiftY
    //  -HeelX *cos(aShiftY) + footC* cos(b), 
    //   footC*sin(b)*sin(aShiftX),  
    //  HeelX*sin(aShiftY) + footC*sin(b)*cos(aShiftY)*cos(aShiftX)

    // (xAnkle0-ax)^2 + (xAnkle1-ay)^2 + (xAnkle2-az)^2 = dLegMax^2
    //ax = -HeelX*cos(Y)-footHeight*sin(Y)   + footC*cos(b)
    //ay = footC*sin(X)*sin(b) - footHeight*sin(X)
    //az = HeelX*sin(Y) + footC*sin(b)*cos(X)*cos(Y) - footHeight*cos(Y)*cos(X)

    //(xLegM0  - cosb*footC)^2 + (xLegM1  - sin(b)*footC*sin(X))^2 + (xLegM2  - sin(b)*footC*cos(X)*cos(Y))^2 = dLegMax^2
    //this eq: p * sinb + q*cosb + r = 0
    double xLegM0 = xAnkle0 + footHeelX*cos(aShiftY) +footHeight*sin(aShiftY);
    double xLegM1 = xAnkle1 + footHeight*sin(aShiftX);
    double xLegM2 = xAnkle2 - footHeelX*sin(aShiftY) + footHeight*cos(aShiftX)*cos(aShiftY);

    double p = -2*footC*xLegM1*sin(aShiftX) -2*footC*xLegM2*cos(aShiftX)*cos(aShiftY);
    double q = -2*footC*xLegM0;

    //////// TODOTODOTODOTODO
    //////// TODOTODOTODOTODO
    double r = xLegM0*xLegM0 + xLegM1*xLegM1 + xLegM2*xLegM2 + footC*footC - dLegMax*dLegMax;



    

    double a = (p*p/q/q + 1);
    double b = 2*p*r/q/q;
    double c = r*r/q/q - 1; 
    double d = b*b-4*a*c;


//With base plane with aShiftY pitchangle
//The rotated ankle position is 
//ax:  footToeX * cos(aShiftY) - cos(a + aFootA + aShiftY)*footC
//ay:  sin(a+aFootA+aShiftY) * footC
  
    if (d > 0){
      double a1 = (-b + sqrt(d))/2/a;
      double a2 = (-b - sqrt(d))/2/a;
      double err1 = fabs(p*a1 + q*sqrt(1-a1*a1)+r);
      double err2 = fabs(p*a2 + q*sqrt(1-a2*a2)+r);
//      double ankle_tilt_angle1 = asin(a1)-afootA;
//      double ankle_tilt_angle2 = asin(a2)-afootA;

      double ankle_tilt_angle1 = asin(a1)-afootA-aShiftY;
      double ankle_tilt_angle2 = asin(a2)-afootA-aShiftY;


      if ((err1<0.0001) && (err2<0.0001)) { //we have two solutions
//        printf("Two lift angle: %.2f %.2f\n",-ankle_tilt_angle1*180/3.1415,-ankle_tilt_angle2*180/3.1415);
        if (fabs(ankle_tilt_angle1)<fabs(ankle_tilt_angle2))
          ankle_tilt_angle = ankle_tilt_angle1;
        else
          ankle_tilt_angle = ankle_tilt_angle2;
      }else{
        if (err1<err2) ankle_tilt_angle = ankle_tilt_angle1;
        else ankle_tilt_angle = ankle_tilt_angle2;
      }

      ankle_tilt_angle = -ankle_tilt_angle; //convert into ankle pitch angle
    }else {
      ankle_tilt_angle = 0;
    }
//    if (ankle_tilt_angle<-45*3.1415/180)  ankle_tilt_angle=-45*3.1415/180;
 
    xLeg[0] = xLeg[0] + footHeelX - footC*cos(-ankle_tilt_angle+afootA);
    xLeg[2] = xLeg[2] - sin(afootA-ankle_tilt_angle)*footC;

/*
  //Compensate the ankle position according to ankle tilt angle
    xLeg[0] = xLeg[0] - footHeelX*cos(aShiftY) - 
              sin(aShiftY-ankle_tilt_angle)*footHeight + footHeelX*cos(aShiftY-ankle_tilt_angle);
    
    xLeg[2] = xLeg[2] + footHeelX*sin(aShiftY) - sin(afootA+ankle_tilt_angle+aShiftY)*footC;
*/
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