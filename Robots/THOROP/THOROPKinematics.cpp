#include "THOROPKinematics.h"
/* 7 DOF */

enum {LEG_LEFT = 0, LEG_RIGHT = 1};
enum {ARM_LEFT = 0, ARM_RIGHT = 1};

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
THOROP_kinematics_forward_l_arm(const double *q) 
{
//New FK for 6-dof arm (pitch-roll-yaw-pitch-yaw-roll)
  Transform t;
  t = t.translateY(shoulderOffsetY)
    .translateZ(shoulderOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, PI/2+q[2], upperArmLength)
    .mDH(PI/2, elbowOffsetX, q[3], 0)
    .mDH(-PI/2, -elbowOffsetX, -PI/2+q[4], lowerArmLength)
    .mDH(-PI/2, 0, -PI/2+q[5], 0)
    .translateX(handOffsetX)
    .translateY(-handOffsetY)
    .translateZ(handOffsetZ);
  return t;
}



  Transform
THOROP_kinematics_forward_r_arm(const double *q) 
{
//New FK for 6-dof arm (pitch-roll-yaw-pitch-yaw-roll)
  Transform t;
  t = t.translateY(-shoulderOffsetY)
    .translateZ(shoulderOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, PI/2+q[2], upperArmLength)
    .mDH(PI/2, elbowOffsetX, q[3], 0)
    .mDH(-PI/2, -elbowOffsetX, -PI/2+q[4], lowerArmLength)
    .mDH(-PI/2, 0, -PI/2+q[5], 0)
    .translateX(handOffsetX)
    .translateY(handOffsetY)
    .translateZ(handOffsetZ);
  return t;
}

  Transform
THOROP_kinematics_forward_l_arm_7(const double *q) 
{
//FK for 7-dof arm (pitch-roll-yaw-pitch-yaw-roll-yaw)
  Transform t;
  t = t.translateY(shoulderOffsetY)
    .translateZ(shoulderOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, PI/2+q[2], upperArmLength)
    .mDH(PI/2, elbowOffsetX, q[3], 0)
    .mDH(-PI/2, -elbowOffsetX, -PI/2+q[4], lowerArmLength)
    .mDH(-PI/2, 0, q[5], 0)
    .mDH(PI/2, 0, q[6], 0)
    .mDH(-PI/2, 0, -PI/2, 0)
    .translateX(handOffsetX)
    .translateY(-handOffsetY)
    .translateZ(handOffsetZ);
  return t;
}


  Transform
THOROP_kinematics_forward_r_arm_7(const double *q) 
{
//New FK for 6-dof arm (pitch-roll-yaw-pitch-yaw-roll)
  Transform t;
  t = t.translateY(-shoulderOffsetY)
    .translateZ(shoulderOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, PI/2+q[2], upperArmLength)
    .mDH(PI/2, elbowOffsetX, q[3], 0)
    .mDH(-PI/2, -elbowOffsetX, -PI/2+q[4], lowerArmLength)
    .mDH(-PI/2, 0, q[5], 0)
    .mDH(PI/2, 0, q[6], 0)
    .mDH(-PI/2, 0, -PI/2, 0)
    .translateX(handOffsetX)
    .translateY(handOffsetY)
    .translateZ(handOffsetZ);

  return t;
}

  Transform
THOROP_kinematics_forward_l_wrist(const double *q) 
{
//FK for 7-dof arm (pitch-roll-yaw-pitch-yaw-roll-yaw)
  Transform t;
  t = t.translateY(shoulderOffsetY)
    .translateZ(shoulderOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, PI/2+q[2], upperArmLength)
    .mDH(PI/2, elbowOffsetX, q[3], 0)
    .mDH(-PI/2, -elbowOffsetX, -PI/2+q[4], lowerArmLength);    
  return t;
}


  Transform
THOROP_kinematics_forward_r_wrist(const double *q) 
{
//New FK for 6-dof arm (pitch-roll-yaw-pitch-yaw-roll)
  Transform t;
  t = t.translateY(-shoulderOffsetY)
    .translateZ(shoulderOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, PI/2+q[2], upperArmLength)
    .mDH(PI/2, elbowOffsetX, q[3], 0)
    .mDH(-PI/2, -elbowOffsetX, -PI/2+q[4], lowerArmLength);    
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

double actlength (double top[], double bot[])
{
  double sum = 0;
  for (int i1=0; i1<3; i1++){
    sum=sum+pow((top[i1]-bot[i1]),2);
  }
  return sqrt(sum);    

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
THOROP_kinematics_inverse_arm(Transform trArm, int arm, const double *qOrg)
{
  //Closed-form inverse kinematics for THOR-OP 6DOF arm
  Transform t;

  double trArm0[3]; //Save initial target position 
  trArm0[0]=trArm(0,3);
  trArm0[1]=trArm(1,3);
  trArm0[2]=trArm(2,3);

  //Getting rid of hand, shoulder offsets
  if (arm==ARM_LEFT){
    t=t.translateZ(-shoulderOffsetZ)
	.translateY(-shoulderOffsetY)
	*trArm
        .translateZ(-handOffsetZ)
        .translateY(handOffsetY)
	.translateX(-handOffsetX);
  }else{

    t=t.translateZ(-shoulderOffsetZ)
	.translateY(shoulderOffsetY)
	*trArm
        .translateZ(-handOffsetZ)
        .translateY(-handOffsetY)
	.translateX(-handOffsetX);
  }

//---------------------------------------------------------------------------
// Calculating elbow pitch from shoulder-wrist distance

  double xWrist[3];
  for (int i = 0; i < 3; i++) xWrist[i]=0;
  t.apply(xWrist);

  double dWrist = xWrist[0]*xWrist[0]+xWrist[1]*xWrist[1]+xWrist[2]*xWrist[2];
  double cElbow = .5*(dWrist-dUpperArm*dUpperArm-dLowerArm*dLowerArm)/(dUpperArm*dLowerArm);
  if (cElbow > 1) cElbow = 1;
  if (cElbow < -1) cElbow = -1;

// SJ: Robot can have TWO elbow pitch values (near elbowPitch==0)
// We are only using the smaller one (arm more bent) 
  double elbowPitch = -acos(cElbow)-aUpperArm-aLowerArm;
//  printf("Elbow pitch: %.2f \n",elbowPitch*180/3.1415);
  
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
//Calculating wrist yaw and roll from shoulder-wrist position 

  //m: transform from shoulder to wrist 
  Transform m;
  m=m.translateX(upperArmLength)
     .translateZ(elbowOffsetX)
     .rotateY(elbowPitch)
     .translateZ(-elbowOffsetX)
     .translateX(lowerArmLength);
  Transform mInv = inv(m);

  //mInvWrist: relative position of shoulder joint from wrist frame before yaw
  double mInvWrist[3];
  for (int i = 0; i < 3; i++) mInvWrist[i]=0;
  mInv.apply(mInvWrist);

  //printf("MinvWrist: %.2f %.2f %.2f\n",mInvWrist[0],mInvWrist[1],mInvWrist[2]);

  //pShoulder: relative position of shoulder joint from wrist frame after yaw/roll
  Transform tInv = inv(t);   
  double pShoulder[3];
  for (int i = 0; i < 3; i++) pShoulder[i]=0;
  tInv.apply(pShoulder);
  //printf("pShoulder: %.2f %.2f %.2f\n",pShoulder[0],pShoulder[1],pShoulder[2]);

  double wristYaw1, wristYaw2 ,s51, s52, c51, c52;


  //Solve equation: pShoulder = RotZ(-q[5])RotX(-q[4])mInvWrist
  // pShoulder[2] + S5 mInvWrist[1] = C5 mInvWrist[2]

  double a,b,c; //coefficients of 2nd order equation
  a = mInvWrist[1]* mInvWrist[1]+ mInvWrist[2]* mInvWrist[2];
  b = mInvWrist[1] * pShoulder[2]; 
  c = pShoulder[2]*pShoulder[2] - mInvWrist[2]*mInvWrist[2];
  //printf("Coeff: %.4f,%.4f,%.4f\n",a,b,c);

  if ((b*b-a*c<0)|| a==0 ) {//NaN handling
   s51=0;
   s52=0;
   wristYaw1 = 0;
   wristYaw2 = 0;
//   printf("NAN\n");
  } 
  else {
    s51= (-b+sqrt(b*b-a*c))/a;
    s52= (-b-sqrt(b*b-a*c))/a;
    if (s51 > 1) s51 = 1;
    if (s51 < -1) s51 = -1;
    if (s52 > 1) s52 = 1;
    if (s52 < -1) s52 = -1;
    //Two possible solutions for wristYaw 
    double wristYaw11 = asin(s51);
    double wristYaw12 = PI-asin(s51);

    double err11= fabs(pShoulder[2] + sin(wristYaw11)*mInvWrist[1] - cos(wristYaw11)*mInvWrist[2]); 
    double err12= fabs(pShoulder[2] + sin(wristYaw12)*mInvWrist[1] - cos(wristYaw12)*mInvWrist[2]); 

    if (err11<0.001) wristYaw1 = wristYaw11;
    else wristYaw1=wristYaw12;


    double wristYaw21 = asin(s52);
    double wristYaw22 = PI-asin(s52);

    double err21= fabs(pShoulder[2] + sin(wristYaw21)*mInvWrist[1] - cos(wristYaw21)*mInvWrist[2]);
    double err22= fabs(pShoulder[2] + sin(wristYaw22)*mInvWrist[1] - cos(wristYaw22)*mInvWrist[2]); 

    if (err21<0.001) wristYaw2 = wristYaw21;
    else wristYaw2=wristYaw22;

  }

/*
  if (arm==ARM_LEFT){
    //Left wrist range: -pi to 0
    //use smaller solution
    //TODO: double check
    s51 = s52;
    wristYaw1 = -PI-wristYaw2;
  }else{
    //Right wrist range: 0 to pi
    //use larger solution
    s52 = s51;
    wristYaw2 = -PI-wristYaw1;
  }
*/

/*
  if (arm==ARM_LEFT){
    //Left wrist range: -pi to 0
    if (wristYaw1<-PI) wristYaw1=wristYaw1+2*PI;
//    if wristYaw1>PI/2 wristYaw1 = PI/2;
    if (wristYaw1>0) wristYaw1 = 0;
    if (wristYaw1<-PI) wristYaw1 = -PI;
    if (wristYaw2<-PI) wristYaw2=wristYaw1+2*PI;
//    if wristYaw2>PI/2 wristYaw2 = PI/2;
    if (wristYaw2>0) wristYaw2 = 0;
    if (wristYaw2<-PI) wristYaw2 = -PI;
    s51=sin(wristYaw1);s52=sin(wristYaw2);

  }else{
    //Right wrist range: -0 to pi
    if (wristYaw1<-PI) wristYaw1=wristYaw1+2*PI;
//    if wristYaw1<-PI/2 wristYaw1 = -PI/2;
    if (wristYaw1<0) wristYaw1 = 0;
    if (wristYaw1>PI) wristYaw1 = PI;

    if (wristYaw2<-PI) wristYaw2=wristYaw1+2*PI;
//    if wristYaw2<-PI/2 wristYaw2 = -PI/2;
    if (wristYaw2<0) wristYaw2 = -0;
    if (wristYaw2>PI) wristYaw2 = PI;
    s51=sin(wristYaw1);s52=sin(wristYaw2);
  }
*/

  c51=cos(wristYaw1);
  c52=cos(wristYaw2);


////////////////////////////////////////////////////////////////////
//Use two wrist yaw joint to solve for 4 possible wrist roll joint
////////////////////////////////////////////////////////////////////

//Move range: -pi/2 to pi/2
//Eq; pShoulder[0] = C6 mInvWrist[0] + S6 (C5*mInvWrist[1]+S5*mInvWrist[2])
// or  (pShoulder[0] - S6 t5)^2 = (1-S6^2) *  mInvWrist[0]^2
//      t5 = C5 * mInvWrist[1] + S5 * mInvWrist[2]

  double t5,s61,s62;
  double wristRoll11, wristRoll12, wristRoll21, wristRoll22;

//Case 1: use WristYaw1 and s51
  t5 = c51 * mInvWrist[1] + s51*mInvWrist[2];

//2nd order equation coefficients
  a = t5*t5 + mInvWrist[0]*mInvWrist[0];
  b = -pShoulder[0]*t5;
  c = pShoulder[0]*pShoulder[0]-mInvWrist[0]*mInvWrist[0];
  if ((b*b-a*c<0)|| a==0 ) {//NaN handling
    wristRoll11 = 0;
    wristRoll12 = 0;
//  printf("NAN at wristRoll\n");
  }
  else {
    s61= (-b+sqrt(b*b-a*c))/a;
    s62= (-b-sqrt(b*b-a*c))/a;
    if (s61 > 1) s61 = 1;
    if (s61 < -1) s61 = -1;
    if (s62 > 1) s62 = 1;
    if (s62 < -1) s62 = -1;

    double wristRoll111=asin(s61);
    double wristRoll112=PI-asin(s61);

    double err11 = fabs(cos(wristRoll111)*mInvWrist[0]+sin(wristRoll111)*t5-pShoulder[0]);
    double err12 = fabs(cos(wristRoll112)*mInvWrist[0]+sin(wristRoll112)*t5-pShoulder[0]);
    if (err11<0.001) wristRoll11 = wristRoll111; 
    else wristRoll11=wristRoll112;


    double wristRoll121=asin(s62);
    double wristRoll122=PI-asin(s62);
    double err21 = fabs(cos(wristRoll121)*mInvWrist[0]+sin(wristRoll121)*t5-pShoulder[0]);
    double err22 = fabs(cos(wristRoll122)*mInvWrist[0]+sin(wristRoll122)*t5-pShoulder[0]);
    if (err21<0.001) wristRoll12 = wristRoll121; 
    else wristRoll12=wristRoll122;

//    wristRoll11=asin(s61);
//    wristRoll12=asin(s62);
  }

//Case 2: use wristYaw2 and s52
  t5 = c52 * mInvWrist[1] + s52*mInvWrist[2];

//2nd order equation coefficients
  a = t5*t5 + mInvWrist[0]*mInvWrist[0];
  b = -pShoulder[0]*t5;
  c = pShoulder[0]*pShoulder[0]-mInvWrist[0]*mInvWrist[0];
  if ((b*b-a*c<0)|| a==0 ) {//NaN handling
    wristRoll21 = 0;
    wristRoll22 = 0;
    // printf("NAN at wristRoll\n");
  }
  else {
    s61= (-b+sqrt(b*b-a*c))/a;
    s62= (-b-sqrt(b*b-a*c))/a;
    if (s61 > 1) s61 = 1;
    if (s61 < -1) s61 = -1;
    if (s62 > 1) s62 = 1;
    if (s62 < -1) s62 = -1;

    double wristRoll111=asin(s61);
    double wristRoll112=PI-asin(s61);

    double err11 = fabs(cos(wristRoll111)*mInvWrist[0]+sin(wristRoll111)*t5-pShoulder[0]);
    double err12 = fabs(cos(wristRoll112)*mInvWrist[0]+sin(wristRoll112)*t5-pShoulder[0]);
    if (err11<0.001) wristRoll21 = wristRoll111; 
    else wristRoll21=wristRoll112;


    double wristRoll121=asin(s62);
    double wristRoll122=PI-asin(s62);
    double err21 = fabs(cos(wristRoll121)*mInvWrist[0]+sin(wristRoll121)*t5-pShoulder[0]);
    double err22 = fabs(cos(wristRoll122)*mInvWrist[0]+sin(wristRoll122)*t5-pShoulder[0]);
    if (err21<0.001) wristRoll22 = wristRoll121; 
    else wristRoll22=wristRoll122;

//    wristRoll21=asin(s61);
//    wristRoll22=asin(s62);
  }


//Two possible solutions
//  printf("Wrist yaw 2 = %.2f \n",-wristYaw*180/3.1415);
//  printf("Wrist roll = %.2f or %.2f\n",wristRoll2*180/3.1415,-wristRoll2*180/3.1415);


//--------------------------------------------------------------------------- 


// Solve shoulder PYR angles for all possible solutions 
// and pick one with smallest postion error 

  double wristYawC[4];
  double wristRollC[4];
  double minError = 999;
  std::vector<double> qArm(6);
  std::vector<double> qArms(25); //returns 4 possible solutions


//We have 4 solution for wrist joints
// (wristYaw1, wristRoll11)
// (wristYaw1, wristRoll12)
// (wristYaw2, wristRoll21)
// (wristYaw2, wristRoll22)

  wristYawC[0]=wristYaw1;
  wristYawC[1]=wristYaw1;
  wristYawC[2]=wristYaw2;
  wristYawC[3]=wristYaw2;
  wristRollC[0]=wristRoll11;
  wristRollC[1]=wristRoll12;
  wristRollC[2]=wristRoll21;
  wristRollC[3]=wristRoll22;

  int solution_no = 0;
  int selected_candidate = 0;

  for (int i=0; i<4; i++){
    Transform m;
    m=m.translateX(upperArmLength)
       .translateZ(elbowOffsetX)
       .rotateY(elbowPitch)
       .translateZ(-elbowOffsetX)
       .translateX(lowerArmLength)
       .rotateX(wristYawC[i]).rotateZ(wristRollC[i]);
    //get P-Y-R rotation matrix
    Transform pyr = t * inv(m);
    double shoulderPitchC = atan2(-pyr(2,0),pyr(0,0));
    double shoulderYawC = atan2(pyr(1,0),
			sqrt(pyr(0,0)*pyr(0,0) + pyr(2,0)*pyr(2,0)));
    double shoulderRollC = atan2(-pyr(1,2),pyr(1,1));

    double armTransformC[6];
    armTransformC[0]=shoulderPitchC; 
    armTransformC[1]=shoulderYawC; 
    armTransformC[2]=shoulderRollC; 
    armTransformC[3]=elbowPitch;
    armTransformC[4]=wristYawC[i];
    armTransformC[5]=wristRollC[i];

    Transform arm_transform_c;

    //Check shoulder angle constraints
    //Elbow should always be out side of the robot

    if (arm==ARM_LEFT){
      if (armTransformC[1]<0) armTransformC[1] = 0;
      if (armTransformC[2]<0) armTransformC[2] = 0;

      arm_transform_c=
	THOROP_kinematics_forward_l_arm(armTransformC); 
    }else{
      if (armTransformC[1]>0) armTransformC[1] = 0;
      if (armTransformC[2]>0) armTransformC[2] = 0;
      arm_transform_c=
	THOROP_kinematics_forward_r_arm(armTransformC); 
    }

    double x_err = trArm0[0] - arm_transform_c(0,3);
    double y_err = trArm0[1] - arm_transform_c(1,3);
    double z_err = trArm0[2] - arm_transform_c(2,3);
    double d_err = x_err*x_err + y_err*y_err + z_err*z_err;

    if (d_err<0.001){
      solution_no++;

      double q0_err = qOrg[0]-armTransformC[0];
      double q1_err = qOrg[1]-armTransformC[1];
      double q2_err = qOrg[2]-armTransformC[2];
      double q3_err = qOrg[3]-armTransformC[3];
      double q4_err = qOrg[4]-armTransformC[4];
      double q5_err = qOrg[5]-armTransformC[5];
      double q_err = q0_err* q0_err + q1_err* q1_err +
 	 	     q2_err* q2_err + q3_err* q3_err +
		     q4_err* q4_err + q5_err* q5_err ;
 
      if (q_err < minError ){
        minError = q_err;
        qArm[0] = armTransformC[0];
        qArm[1] = armTransformC[1];
        qArm[2] = armTransformC[2];
        qArm[3] = armTransformC[3];
        qArm[4] = armTransformC[4];
        qArm[5] = armTransformC[5];
      }
    }
  }
if (arm==ARM_LEFT) 
//printf("Solutions: %d\n",solution_no);
  if (solution_no==0){
    qArm[0] = qOrg[0];
    qArm[1] = qOrg[1];
    qArm[2] = qOrg[2];
    qArm[3] = qOrg[3];
    qArm[4] = qOrg[4];
    qArm[5] = qOrg[5];
  }
  return qArm;
}





//This function calcaulates IK for THOR-OP 6DOF arm
//Given the wrist joint position and shoulder yaw angle
//Main purpose:  quadruped locomotion

  std::vector<double>
THOROP_kinematics_inverse_wrist(Transform trArm, 
	double shoulderYaw, int arm)
{
  //Closed-form inverse kinematics for THOR-OP 6DOF arm
  Transform t;

  //Getting rid of shoulder offsets
  if (arm==ARM_LEFT){
    t=t.translateZ(-shoulderOffsetZ)
	.translateY(-shoulderOffsetY)
	*trArm;
  }else{

    t=t.translateZ(-shoulderOffsetZ)
	.translateY(shoulderOffsetY)
	*trArm;
  }

  double wPos[3]; 
  wPos[0]=t(0,3);
  wPos[1]=t(1,3);
  wPos[2]=t(2,3);

//---------------------------------------------------------------------------
// Calculating elbow pitch from shoulder-wrist distance

  double xWrist[3];
  for (int i = 0; i < 3; i++) xWrist[i]=0;
  t.apply(xWrist);

  double dWrist = xWrist[0]*xWrist[0]+xWrist[1]*xWrist[1]+xWrist[2]*xWrist[2];
  double cElbow = .5*(dWrist-dUpperArm*dUpperArm-dLowerArm*dLowerArm)/(dUpperArm*dLowerArm);
  if (cElbow > 1) cElbow = 1;
  if (cElbow < -1) cElbow = -1;
// SJ: Robot can have TWO elbow pitch values (near elbowPitch==0)
// We are only using the smaller one (arm more bent) 
  double elbowPitch = -acos(cElbow)-aUpperArm-aLowerArm;

  //m: transform from shoulder yaw to wrist 
  Transform m;
  m=m.rotateX(shoulderYaw)
     .translateX(upperArmLength)
     .translateZ(elbowOffsetX)
     .rotateY(elbowPitch)
     .translateZ(-elbowOffsetX)
     .translateX(lowerArmLength);

  double a,b,c;

  //Ry(shoulderPitch) * Rz(ShoulderRoll) * m * (0 0 0 1)^T= Target Position
  
  //s1x + c1z = m3
  //or (z^2+x^2) s1^2 + (2m3z) s1 + (m3^2-x^2) = 0
  double s11=0.0, s12=0.0, shoulderPitch=0.0;

  a = wPos[2]*wPos[2] + wPos[0]*wPos[0];
  b = -m(2,3)*wPos[0];
  c = m(2,3)*m(2,3) - wPos[2]*wPos[2];

  if ((b*b-a*c<0)|| a==0 ) {//NaN handling
    shoulderPitch = 0.0;
  }else{
    double s11= (-b+sqrt(b*b-a*c))/a;
    double s12= (-b-sqrt(b*b-a*c))/a;
    if (s11<-1) s11=1; 
    if (s11>1) s11=1;
    if (s12<-1) s12=1; 
    if (s12>1) s12=1;
    double c11 = sqrt(1-s11*s11); //pitch is between -pi/2 and pi/2
    double c12 = sqrt(1-s12*s12);
    double error1 = fabs(s11* wPos[0] + c11*wPos[2]- m(2,3));
    double error2 = fabs(s12* wPos[0] + c12*wPos[2]- m(2,3));
    if (error1<error2) shoulderPitch = asin(s11);
    else shoulderPitch = asin(s12);
  }

  double c21=0.0, c22=0.0, shoulderRoll=0.0;
  // m2 = k* s2 + y* c2 , k = s1 z -  c1 x
  // or (y^2 + k^2) s2^2 - 2 m2 k s2 + m2^2-y^2 = 0

  double k = sin(shoulderPitch)*wPos[2] - cos(shoulderPitch)*wPos[0];
  a = wPos[1]*wPos[1] + k*k;
  b = - m(1,3)*k;
  c = m(1,3)*m(1,3) - wPos[1]*wPos[1]; 

  if ((b*b-a*c<0)|| a==0 ) {//NaN handling
    shoulderRoll = 0.0;
  }else{
    double s21,s22,c21,c22;
    s21= (-b+sqrt(b*b-a*c))/a;
    s22= (-b-sqrt(b*b-a*c))/a;
    if (s21<-1) s21=1; 
    if (s21>1) s21=1;
    if (s22<-1) s22=1; 
    if (s22>1) s22=1;
    c21 = sqrt(1-c21*c21);
    c22 = sqrt(1-c22*c22);
    double error1 = fabs(c21* wPos[1] + s21*k- m(1,3));
    double error2 = fabs(c22* wPos[1] + s22*k- m(1,3));
    if (error1<error2)	shoulderRoll = asin(s21);
    else	shoulderRoll = asin(s22);
  }

  std::vector<double> qArm(6);
  qArm[0] = shoulderPitch;
  qArm[1] = shoulderRoll;
  qArm[2] = shoulderYaw;
  qArm[3] = elbowPitch;
  qArm[4] = 0;
  qArm[5] = 0;

  return qArm;
}





  std::vector<double>
THOROP_kinematics_inverse_wrist(Transform trWrist, int arm, const double *qOrg, double shoulderYaw) {
  //calculate soulder and elbow angle given wrist POSITION
  // Shoulder yaw angle is given

  Transform t;

  //Getting rid of hand, shoulder offsets
  if (arm==ARM_LEFT){
    t=t.translateZ(-shoulderOffsetZ)
    .translateY(-shoulderOffsetY)
    .translateY(-shoulderOffsetX)
  *trWrist;
  }else{
    t=t.translateZ(-shoulderOffsetZ)
    .translateY(shoulderOffsetY)
    .translateY(shoulderOffsetX)
  *trWrist;
  }

//---------------------------------------------------------------------------
// Calculating elbow pitch from shoulder-wrist distance
//---------------------------------------------------------------------------
     
  double xWrist[3];
  for (int i = 0; i < 3; i++) xWrist[i]=0;
  t.apply(xWrist);

  double dWrist = xWrist[0]*xWrist[0]+xWrist[1]*xWrist[1]+xWrist[2]*xWrist[2];
  double cElbow = .5*(dWrist-dUpperArm*dUpperArm-dLowerArm*dLowerArm)/(dUpperArm*dLowerArm);
  if (cElbow > 1) cElbow = 1;
  if (cElbow < -1) cElbow = -1;

  // SJ: Robot can have TWO elbow pitch values (near elbowPitch==0)
  // We are only using the smaller one (arm more bent) 
  double elbowPitch = -acos(cElbow)-aUpperArm-aLowerArm;  

//---------------------------------------------------------------------------
// Calculate arm shoulder pitch and roll given the wrist position
//---------------------------------------------------------------------------

  //Transform m: from shoulder yaw to wrist 
  Transform m;
  m=m.rotateX(shoulderYaw)
     .translateX(upperArmLength)
     .translateZ(elbowOffsetX)
     .rotateY(elbowPitch)
     .translateZ(-elbowOffsetX)
     .translateX(lowerArmLength);
  //Now we solve the equation
  //RotY(shoulderPitch)*RotZ(ShoulderRoll)*m*(0 0 0 1)T = xWrist
  
  //Solve shoulder roll first
  //sin(shoulderRoll)*m[0][3] + cos(shoulderRoll)*m[1][3] = xWrist[1]
  //Solve for sin (roll limited to -pi/2 to pi/2)
  
  double a,b,c;
  double shoulderPitch, shoulderRoll;
  double err1,err2;

  a = m(0,3)*m(0,3) + m(1,3)*m(1,3);
  b = -m(0,3)*xWrist[1];
  c = xWrist[1]*xWrist[1] - m(1,3)*m(1,3);
  if ((b*b-a*c<0)|| a==0 ) {//NaN handling
    shoulderRoll = 0;
  }
  else {
    double c21,c22,s21,s22;
    s21= (-b+sqrt(b*b-a*c))/a;
    s22= (-b-sqrt(b*b-a*c))/a;
    if (s21 > 1) s21 = 1;
    if (s21 < -1) s21 = -1;
    if (s22 > 1) s22 = 1;
    if (s22 < -1) s22 = -1;
    double shoulderRoll1 = asin(s21);
    double shoulderRoll2 = asin(s22);
    err1 = s21*m(0,3)+cos(shoulderRoll1)*m(1,3)-xWrist[1];
    err2 = s22*m(0,3)+cos(shoulderRoll2)*m(1,3)-xWrist[1];
    if (err1*err1<err2*err2) shoulderRoll = shoulderRoll1;
    else shoulderRoll = shoulderRoll2;
  }

//Now we know shoulder Roll and Yaw
//Solve for shoulder Pitch
//Eq 1: c1(c2*m[0][3]-s2*m[1][3])+s1*m[2][3] = xWrist[0]
//Eq 2: -s1(c2*m[0][3]-s2*m[1][3])+c1*m[2][3] = xWrist[2]
//OR
// s1*t1 + c1*m[2][3] = xWrist[2]
// -c1*t1 + s1*m[2][3] = xWrist[0]
  // c1 = (m[2][3] xWrist[2] - t1 xWrist[0])/ (m[2][3]^2 - t1^2 )
  // s1 = (m[2][3] xWrist[0] + t1 xWrist[2])/ (m[2][3]^2 + t1^2 )

  double s2 = sin(shoulderRoll);
  double c2 = cos(shoulderRoll);
  double t1 = -c2 * m(0,3) + s2 * m(1,3);
 
  double c1 = (m(2,3)*xWrist[2]-t1*xWrist[0]) /(m(2,3)*m(2,3) + t1*t1);
  double s1 = (m(2,3)*xWrist[0]+t1*xWrist[2]) /(m(2,3)*m(2,3) + t1*t1);

  shoulderPitch = atan2(s1,c1);

  //return joint angles
  std::vector<double> qArm(7);
  qArm[0] = shoulderPitch;
  qArm[1] = shoulderRoll;
  qArm[2] = shoulderYaw;
  qArm[3] = elbowPitch;

  qArm[4] = qOrg[4];
  qArm[5] = qOrg[5];
  qArm[6] = qOrg[6];

  return qArm;
}





  std::vector<double>
THOROP_kinematics_inverse_arm_7(Transform trArm, int arm, const double *qOrg, double shoulderYaw) {
  // Closed-form inverse kinematics for THOR-OP 7DOF arm
  // (pitch-roll-yaw-pitch-yaw-roll-yaw)
  // Shoulder yaw angle is given

  Transform t;

  //Getting rid of hand, shoulder offsets
  if (arm==ARM_LEFT){
    t=t.translateZ(-shoulderOffsetZ)
  	.translateY(-shoulderOffsetY)
    .translateY(-shoulderOffsetX)
	*trArm
    .translateZ(-handOffsetZ)
    .translateY(handOffsetY)
	  .translateX(-handOffsetX);
  }else{
    t=t.translateZ(-shoulderOffsetZ)
    .translateY(shoulderOffsetY)
    .translateY(shoulderOffsetX)
	*trArm
    .translateZ(-handOffsetZ)
    .translateY(-handOffsetY)
  	.translateX(-handOffsetX);
  }

  Transform trArmRot; //Only the rotation part of trArm
  trArmRot = trArmRot*trArm
	   .translateZ(-trArmRot(2,3))
	   .translateY(-trArmRot(1,3))
	   .translateX(-trArmRot(0,3));

//---------------------------------------------------------------------------
// Calculating elbow pitch from shoulder-wrist distance
//---------------------------------------------------------------------------

  double xWrist[3];
  for (int i = 0; i < 3; i++) xWrist[i]=0;
  t.apply(xWrist);

  double dWrist = xWrist[0]*xWrist[0]+xWrist[1]*xWrist[1]+xWrist[2]*xWrist[2];
  double cElbow = .5*(dWrist-dUpperArm*dUpperArm-dLowerArm*dLowerArm)/(dUpperArm*dLowerArm);
  if (cElbow > 1) cElbow = 1;
  if (cElbow < -1) cElbow = -1;

  // SJ: Robot can have TWO elbow pitch values (near elbowPitch==0)
  // We are only using the smaller one (arm more bent) 
  double elbowPitch = -acos(cElbow)-aUpperArm-aLowerArm;  

//---------------------------------------------------------------------------
// Calculate arm shoulder pitch and roll given the wrist position
//---------------------------------------------------------------------------

  //Transform m: from shoulder yaw to wrist 
  Transform m;
  m=m.rotateX(shoulderYaw)
     .translateX(upperArmLength)
     .translateZ(elbowOffsetX)
     .rotateY(elbowPitch)
     .translateZ(-elbowOffsetX)
     .translateX(lowerArmLength);
  //Now we solve the equation
  //RotY(shoulderPitch)*RotZ(ShoulderRoll)*m*(0 0 0 1)T = xWrist
  
  //Solve shoulder roll first
  //sin(shoulderRoll)*m[0][3] + cos(shoulderRoll)*m[1][3] = xWrist[1]
  //Solve for sin (roll limited to -pi/2 to pi/2)
  
  double a,b,c;
  double shoulderPitch, shoulderRoll;
  double err1,err2;

  a = m(0,3)*m(0,3) + m(1,3)*m(1,3);
  b = -m(0,3)*xWrist[1];
  c = xWrist[1]*xWrist[1] - m(1,3)*m(1,3);
  if ((b*b-a*c<0)|| a==0 ) {//NaN handling
    shoulderRoll = 0;
  }
  else {
    double c21,c22,s21,s22;
    s21= (-b+sqrt(b*b-a*c))/a;
    s22= (-b-sqrt(b*b-a*c))/a;
    if (s21 > 1) s21 = 1;
    if (s21 < -1) s21 = -1;
    if (s22 > 1) s22 = 1;
    if (s22 < -1) s22 = -1;
    double shoulderRoll1 = asin(s21);
    double shoulderRoll2 = asin(s22);
    err1 = s21*m(0,3)+cos(shoulderRoll1)*m(1,3)-xWrist[1];
    err2 = s22*m(0,3)+cos(shoulderRoll2)*m(1,3)-xWrist[1];
    if (err1*err1<err2*err2) shoulderRoll = shoulderRoll1;
    else shoulderRoll = shoulderRoll2;
  }

//Now we know shoulder Roll and Yaw
//Solve for shoulder Pitch
//Eq 1: c1(c2*m[0][3]-s2*m[1][3])+s1*m[2][3] = xWrist[0]
//Eq 2: -s1(c2*m[0][3]-s2*m[1][3])+c1*m[2][3] = xWrist[2]
//OR
// s1*t1 + c1*m[2][3] = xWrist[2]
// -c1*t1 + s1*m[2][3] = xWrist[0]
  // c1 = (m[2][3] xWrist[2] - t1 xWrist[0])/ (m[2][3]^2 - t1^2 )
  // s1 = (m[2][3] xWrist[0] + t1 xWrist[2])/ (m[2][3]^2 + t1^2 )

  double s2 = sin(shoulderRoll);
  double c2 = cos(shoulderRoll);
  double t1 = -c2 * m(0,3) + s2 * m(1,3);
 
  double c1 = (m(2,3)*xWrist[2]-t1*xWrist[0]) /(m(2,3)*m(2,3) + t1*t1);
  double s1 = (m(2,3)*xWrist[0]+t1*xWrist[2]) /(m(2,3)*m(2,3) + t1*t1);

  shoulderPitch = atan2(s1,c1);

//---------------------------------------------------------------------------
// Now we know shoulder pich, roll, yaw and elbow pitch
// Calc the final transform for the wrist based on rotation alone
//---------------------------------------------------------------------------

  Transform tRot;
  tRot = tRot
     .rotateY(shoulderPitch)
     .rotateZ(shoulderRoll)
     .rotateX(shoulderYaw)
     .rotateY(elbowPitch);

  Transform tInvRot;

  tInvRot = tInvRot
	.rotateY(-elbowPitch)
	.rotateX(-shoulderYaw)
	.rotateZ(-shoulderRoll)
	.rotateY(-shoulderPitch);

  //Now we solve  
  // tRot * RotX(wristYaw)*RotZ(wristRoll)*RotX(wristYaw2) = trArmRot
  // or inv(tRot) * trArmRot = RotX RotZ RotX

//  Transform rotWrist = inv(tRot) * trArmRot;
  Transform rotWrist = tInvRot * trArmRot;

  double wristYaw_a, wristRoll_a, wristYaw2_a,
        wristYaw_b, wristRoll_b, wristYaw2_b;

  //Two solutions
  wristRoll_a = acos(rotWrist(0,0)); //0 to pi
  double swa = sin(wristRoll_a);
  wristYaw_a = atan2 (rotWrist(2,0)*swa,rotWrist(1,0)*swa);
  wristYaw2_a = atan2 (rotWrist(0,2)*swa,-rotWrist(0,1)*swa);

  //Filpped position
  wristRoll_b = -wristRoll_a; //-pi to 0
  double swb = sin(wristRoll_b);
  wristYaw_b = atan2 (rotWrist(2,0)*swb,rotWrist(1,0)*swb);  
  wristYaw2_b = atan2 (rotWrist(0,2)*swb,-rotWrist(0,1)*swb);

  //singular point: just use current angles
  if(swa<0.00001){
    wristYaw_a = qOrg[4];    
    wristRoll_a = qOrg[5];
    wristYaw2_a = qOrg[6];
    wristYaw_b = qOrg[4];
    wristRoll_b = qOrg[5];
    wristYaw2_b = qOrg[6];
  } 

  //Select the closest solution to current joint angle  
  bool select_a = false;
  double err_a = fmodf(qOrg[4] - wristYaw_a+5*PI, 2*PI) - PI;
  double err_b = fmodf(qOrg[4] - wristYaw_b+5*PI, 2*PI) - PI;
  if (err_a*err_a<err_b*err_b)   select_a=true;
  
  std::vector<double> qArm(7);
  qArm[0] = shoulderPitch;
  qArm[1] = shoulderRoll;
  qArm[2] = shoulderYaw;
  qArm[3] = elbowPitch;

  if (select_a==true) {
    qArm[4] = wristYaw_a;
    qArm[5] = wristRoll_a;
    qArm[6] = wristYaw2_a;
  }else{
    qArm[4] = wristYaw_b;
    qArm[5] = wristRoll_b;
    qArm[6] = wristYaw2_b;
  }
  return qArm;
}


  std::vector<double>
THOROP_kinematics_inverse_l_arm(Transform trArm, const double *qOrg)
{
  return THOROP_kinematics_inverse_arm(trArm, ARM_LEFT, qOrg);
}

  std::vector<double>
THOROP_kinematics_inverse_r_arm(Transform trArm, const double *qOrg)
{
  return THOROP_kinematics_inverse_arm(trArm, ARM_RIGHT, qOrg);
}


  std::vector<double>
THOROP_kinematics_inverse_l_arm_7(Transform trArm, const double *qOrg, double shoulderYaw)
{
  return THOROP_kinematics_inverse_arm_7(trArm, ARM_LEFT, qOrg, shoulderYaw);
}

  std::vector<double>
THOROP_kinematics_inverse_r_arm_7(Transform trArm, const double *qOrg,double shoulderYaw)
{
  return THOROP_kinematics_inverse_arm_7(trArm, ARM_RIGHT, qOrg, shoulderYaw);
}




  std::vector<double>
THOROP_kinematics_inverse_l_wrist(Transform trWrist, const double *qOrg, double shoulderYaw)
{
  return THOROP_kinematics_inverse_wrist(trWrist, ARM_LEFT, qOrg, shoulderYaw);
}

  std::vector<double>
THOROP_kinematics_inverse_r_wrist(Transform trWrist,const double *qOrg, double shoulderYaw)
{
  return THOROP_kinematics_inverse_wrist(trWrist, ARM_RIGHT, qOrg, shoulderYaw);
}



  std::vector<double>
THOROP_kinematics_inverse_leg(Transform trLeg, int leg)
{
  std::vector<double> qLeg(6);
  Transform trInvLeg = inv(trLeg);

  // Hip Offset vector in Torso frame
  double xHipOffset[3];
  if (leg == LEG_LEFT) {
    xHipOffset[0] = 0;
    xHipOffset[1] = hipOffsetY;
    xHipOffset[2] = -hipOffsetZ;
  }
  else {
    xHipOffset[0] = 0;
    xHipOffset[1] = -hipOffsetY;
    xHipOffset[2] = -hipOffsetZ;
  }

  // Hip Offset in Leg frame
  double xLeg[3];
  for (int i = 0; i < 3; i++)
    xLeg[i] = xHipOffset[i];
  trInvLeg.apply(xLeg);
  xLeg[2] -= footHeight;

  // Knee pitch
  double dLeg = xLeg[0]*xLeg[0] + xLeg[1]*xLeg[1] + xLeg[2]*xLeg[2];

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
  return qLeg;
}

  std::vector<double>
THOROP_kinematics_inverse_l_leg(Transform trLeg)
{
  return THOROP_kinematics_inverse_leg(trLeg, LEG_LEFT);
}

  std::vector<double>
THOROP_kinematics_inverse_r_leg(Transform trLeg)
{
  return THOROP_kinematics_inverse_leg(trLeg, LEG_RIGHT);
}
