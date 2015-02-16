#include "THOROPKinematics.h"
/* 7 DOF */

enum {LEG_LEFT = 0, LEG_RIGHT = 1};
enum {ARM_LEFT = 0, ARM_RIGHT = 1};

/*
void printTransform(Transform tr) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      printf("%.4g ", tr(i,j));
    }
    printf("\n");
  }
  printf("\n");
}

void printVector(std::vector<double> v) {
  for (int i = 0; i < v.size(); i++) {
    printf("%.4g\n", v[i]);
  }
  printf("\n");
}
*/

  std::vector<double>
THOROP_kinematics_forward_joints(const double *r)
{
  /* forward kinematics to convert servo positions to joint angles */
  std::vector<double> q(23);
  for (int i = 0; i < 23; i++) {
    q[i] = r[i];
  }
  return q;
}

//DH transform params: (alpha, a, theta, d)

  Transform
THOROP_kinematics_forward_head(const double *q)
{
  Transform t;
  t = t.translateZ(neckOffsetZ)
    .mDH(0, 0, q[0], 0)
    .mDH(-PI/2, 0, -PI/2+q[1], 0)
    .rotateX(PI/2).rotateY(PI/2);
  return t;
}

  Transform
THOROP_kinematics_forward_l_leg(const double *q)
{
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

  Transform
THOROP_kinematics_forward_r_leg(const double *q)
{
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


  std::vector<double>
THOROP_kinematics_inverse_joints(const double *q)  //dereks code to write
{
  /* inverse kinematics to convert joint angles to servo positions */
  std::vector<double> r(23);
  for (int i = 0; i < 23; i++) {
    r[i] = q[i];
  }
  return r;
}



  std::vector<double>
THOROP_kinematics_calculate_foot_lift(Transform trLeg, int leg)  
{
  std::vector<double> qFootLift(2);
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
  double dLegMax = dTibia + dThigh;

  double cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);

  //Automatic heel lift when IK limit is reached
  double footCompZ = 0;
  double ankle_tilt_angle = 0;

  double footC = sqrt(footHeight*footHeight + footToeX*footToeX);
  double afootA = asin(footHeight/footC);
  double xLeg0Mod = xLeg[0] - footToeX;

  double footHeelC = sqrt(footHeight*footHeight + footHeelX*footHeelX);
  double afootHeel = asin(footHeight/footHeelC);
  double xLeg0ModHeel = xLeg[0] + footHeelX;

  double p,q,r,a,b,c,d;
  double heel_lift = 0;
  double toe_lift = 0;

  if (dLeg>dLegMax*dLegMax) {
//    printf("xLeg: %.3f,%.3f,%.3f\n",xLeg[0],xLeg[1],xLeg[2]);
   
    //Calculate the amount of heel lift

    // then rotated ankle position (ax,az) is footToeX-cos(a+aFootA)*footC, sin(a+aFootA)*footC
    // or footToeX-cosb*footC, sinb *footC
    // then 
    // (xLeg[0]-ax)^2 + xLeg[1]^2 + (xLeg[2]-az)^2 = dLegMax^2
    // or (xLeg0Mod + cosb*footC)^2 + xLeg[1]^2 + (xLeg[2]-sinb*footC)^2 = dLegMax^2
    //this eq: p * sinb + q*cosb + r = 0

    p = -2*footC*xLeg[2];
    q = 2*footC*xLeg0Mod;
    r = xLeg0Mod*xLeg0Mod + xLeg[1]*xLeg[1] +xLeg[2]*xLeg[2] - dLegMax*dLegMax +footC*footC;
    a = (p*p/q/q + 1);
    b = 2*p*r/q/q;
    c = r*r/q/q - 1; 
    d = b*b-4*a*c;

    if (d > 0){
      double a1 = (-b + sqrt(d))/2/a;
      double a2 = (-b - sqrt(d))/2/a;
      double err1 = fabs(p*a1 + q*sqrt(1-a1*a1)+r);
      double err2 = fabs(p*a2 + q*sqrt(1-a2*a2)+r);
      double ankle_tilt_angle1 = asin(a1)-afootA;
      double ankle_tilt_angle2 = asin(a2)-afootA;
      if ((err1<0.0001) && (err2<0.0001)) { //we have two solutions
        if (fabs(ankle_tilt_angle1)<fabs(ankle_tilt_angle2))
          heel_lift = ankle_tilt_angle1;
        else
          heel_lift = ankle_tilt_angle2;
      }else{
        if (err1<err2) heel_lift = ankle_tilt_angle1;
        else heel_lift = ankle_tilt_angle2;
      }
    }else{
      heel_lift = -1; //not possible
    }

    //Calculate the amount of toe lift
   
    p = -2*footHeelC*xLeg[2];
    q = -2*footHeelC*xLeg0ModHeel;
    r = xLeg0ModHeel*xLeg0ModHeel + xLeg[1]*xLeg[1] +xLeg[2]*xLeg[2] - dLegMax*dLegMax +footHeelC*footHeelC;
    a = (p*p/q/q + 1);
    b = 2*p*r/q/q;
    c = r*r/q/q - 1; 
    d = b*b-4*a*c;

    if (d > 0){
      double a1 = (-b + sqrt(d))/2/a;
      double a2 = (-b - sqrt(d))/2/a;
      double err1 = fabs(p*a1 + q*sqrt(1-a1*a1)+r);
      double err2 = fabs(p*a2 + q*sqrt(1-a2*a2)+r);
      double ankle_tilt_angle1 = - (asin(a1)-afootHeel);
      double ankle_tilt_angle2 = - (asin(a2)-afootHeel);
      if ((err1<0.0001) && (err2<0.0001)) { //we have two solutions
        if (fabs(ankle_tilt_angle1)<fabs(ankle_tilt_angle2)) toe_lift = ankle_tilt_angle1;
        else toe_lift = ankle_tilt_angle2;
      }else{
        if (err1<err2) toe_lift = ankle_tilt_angle1;
        else toe_lift = ankle_tilt_angle2;
      }
    }else{
      toe_lift = -1; //not possible
    }
  }
  qFootLift[0] = heel_lift;
  qFootLift[1] = toe_lift;  
  return qFootLift;
}




  std::vector<double>
THOROP_kinematics_inverse_leg_tilt(Transform trLeg,double footTilt, int leg)
{
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

  double footToeC = sqrt(footHeight*footHeight + footToeX*footToeX);
  double aFootToe = asin(footHeight/footToeC);
  double footHeelC = sqrt(footHeight*footHeight + footHeelX*footHeelX);
  double aFootHeel = asin(footHeight/footHeelC);

  //Automatic heel lift when IK limit is reached
  double footCompZ = 0;

  if (footTilt>0){ //This means heel lift
    xLeg[0] = xLeg[0] - footToeX + cos(aFootToe+footTilt)* footToeC;
    xLeg[2] = xLeg[2] - sin(aFootToe+footTilt)*footToeC;
  }else{ //This means toe lift
    xLeg[0] = xLeg[0] + footHeelX - cos(aFootHeel-footTilt)*footHeelC;
    xLeg[2] = xLeg[2] - sin(aFootHeel-footTilt)*footHeelC;
  }

// Knee pitch
  double dLeg = xLeg[0]*xLeg[0] + xLeg[1]*xLeg[1] + xLeg[2]*xLeg[2]; 
  double dLegMax = dTibia + dThigh;
  double cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);

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

  qLeg[4] = qLeg[4]+footTilt;
  return qLeg;
}


  std::vector<double>
THOROP_kinematics_inverse_leg(Transform trLeg, int leg, double aShiftX, double aShiftY)
{
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
//  double dLeg = xLeg[0]*xLeg[0] + xLeg[1]*xLeg[1] + (xLeg[2]-footHeight/cos(aShiftY))*(xLeg[2]-footHeight/cos(aShiftY));
  
  double dLegMax = dTibia + dThigh;

  double cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);

  //Automatic heel lift when IK limit is reached
  double footCompZ = 0;
  double ankle_tilt_angle = 0;

  double footC = sqrt(footHeight*footHeight + footToeX*footToeX);
  double afootA = asin(footHeight/footC);

  double xLeg0Mod = xLeg[0] - footToeX;

//  double xLeg0Mod = xLeg[0] - footToeX*cos(aShiftY);
 



  double footHeelC = sqrt(footHeight*footHeight + footHeelX*footHeelX);
  double afootHeel = asin(footHeight/footC);
  double xLeg0ModHeel = xLeg[0] + footHeelX;


//TODOTODOTODO: automatic heel lift for UNEVEN surface



  //if (dLeg>dLegMax*dLegMax) {
  if (false){




//    printf("xLeg: %.3f,%.3f,%.3f\n",xLeg[0],xLeg[1],xLeg[2]);
   
    //Calculate the amount of heel lift
    // then rotated ankle position (ax,az) is footToeX-cos(a+aFootA)*footC, sin(a+aFootA)*footC
    // or footToeX-cosb*footC, sinb *footC
    // then 
    // (xLeg[0]-ax)^2 + xLeg[1]^2 + (xLeg[2]-az)^2 = dLegMax^2
    // or (xLeg0Mod + cosb*footC)^2 + xLeg[1]^2 + (xLeg[2]-sinb*footC)^2 = dLegMax^2
    //this eq: p * sinb + q*cosb + r = 0

    double p = -2*footC*xLeg[2];
    double q = 2*footC*xLeg0Mod;
    double r = xLeg0Mod*xLeg0Mod + xLeg[1]*xLeg[1] +xLeg[2]*xLeg[2] - dLegMax*dLegMax +footC*footC;

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
  }else {
      ankle_tilt_angle = 0;
    }
    if (ankle_tilt_angle>30*3.1415/180)  ankle_tilt_angle=30*3.1415/180;



    //Compensate the ankle position according to ankle tilt angle
//    xLeg[0] = xLeg[0] - (sin(ankle_tilt_angle)*footHeight + (1-cos(ankle_tilt_angle))*footToeX);
//    xLeg[2] = xLeg[2] - sin(afootA+ankle_tilt_angle)*footC;

    xLeg[0] = xLeg[0] + footToeX*cos(aShiftY) - 
              sin(aShiftY+ankle_tilt_angle)*footHeight - footToeX*cos(aShiftY+ankle_tilt_angle);
    
    xLeg[2] = xLeg[2] + footToeX*sin(aShiftY)
    - sin(afootA+ankle_tilt_angle+aShiftY)*footC;

    dLeg = xLeg[0]*xLeg[0] + xLeg[1]*xLeg[1] + xLeg[2]*xLeg[2];
    cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);
  }else{    

    xLeg[2] -= footHeight;
  }


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

  qLeg[4] = qLeg[4]+ankle_tilt_angle;
  return qLeg;
}


double 
THOROP_kinematics_calculate_knee_height(const double *q){
//We calculate the knee height assuming the foot is on the ground


  Transform trKnee; //Only the rotation part of trArm

  trKnee = trKnee
     .translateZ(footHeight)
     .rotateX(-q[5])
     .rotateY(-q[4])
     .translateZ(tibiaLength)
     .translateX(kneeOffsetX);
  
  return  trKnee(2,3);
} 


  std::vector<double>
THOROP_kinematics_inverse_l_leg(Transform trLeg, double aShiftX, double aShiftY)
{return THOROP_kinematics_inverse_leg(trLeg, LEG_LEFT,aShiftX,  aShiftY);}

  std::vector<double>
THOROP_kinematics_inverse_r_leg(Transform trLeg, double aShiftX, double aShiftY)
{return THOROP_kinematics_inverse_leg(trLeg, LEG_RIGHT, aShiftX,  aShiftY);}