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
/*
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
*/
  Transform
THOROP_kinematics_forward_l_arm_7(const double *q, double bodyPitch, const double *qWaist) 
{
//FK for 7-dof arm (pitch-roll-yaw-pitch-yaw-roll-yaw)
  Transform t;
  t = t
    .rotateY(bodyPitch)
    .rotateZ(qWaist[0]).rotateY(qWaist[1])
    .translateY(shoulderOffsetY)
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
THOROP_kinematics_forward_r_arm_7(const double *q, double bodyPitch, const double *qWaist)  
{
//New FK for 6-dof arm (pitch-roll-yaw-pitch-yaw-roll)
  Transform t;
  t = t
    .rotateY(bodyPitch)
    .rotateZ(qWaist[0]).rotateY(qWaist[1])
    .translateY(-shoulderOffsetY)
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
THOROP_kinematics_forward_l_wrist(const double *q, double bodyPitch, const double *qWaist) 
{
//FK for 7-dof arm (pitch-roll-yaw-pitch-yaw-roll-yaw)
  Transform t;
  t = t
    .rotateY(bodyPitch)
    .rotateZ(qWaist[0]).rotateY(qWaist[1])
    .translateY(shoulderOffsetY)
    .translateZ(shoulderOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, PI/2+q[2], upperArmLength)
    .mDH(PI/2, elbowOffsetX, q[3], 0)
    .mDH(-PI/2, -elbowOffsetX, -PI/2+q[4], lowerArmLength);    
  return t;
}


  Transform
THOROP_kinematics_forward_r_wrist(const double *q, double bodyPitch, const double *qWaist) 
{
//New FK for 6-dof arm (pitch-roll-yaw-pitch-yaw-roll)
  Transform t;
  t = t
    .rotateY(bodyPitch)
    .rotateZ(qWaist[0]).rotateY(qWaist[1])
    .translateY(-shoulderOffsetY)
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

//Get the COM and total mass of the upper body

  std::vector<double>
THOROP_kinematics_com_upperbody(
    const double *qWaist,
    const double *qLArm,
    const double *qRArm,
    double bodyPitch,
    double mLHand,
    double mRHand)  
{



//SJ: THEY ARE MESSED UP


  /* inverse kinematics to convert joint angles to servo positions */
  std::vector<double> r(4);

  //Now we use PELVIS frame as the default frame
  //As the IMU are located there

  Transform tPelvis, tTorso, 
            tLShoulder, tLElbow, tLWrist, tLHand,
            tRShoulder, tRElbow, tRWrist, tRHand,

            tPelvisCOM, tTorsoCOM,
            tLUArmCOM, tLElbowCOM, tLLArmCOM, tLWristCOM, tLHandCOM,
            tRUArmCOM, tRElbowCOM, tRLArmCOM, tRWristCOM, tRHandCOM;
  
  tPelvis = tPelvis
    .rotateY(bodyPitch);

  tTorso = trcopy(tPelvis)
    .rotateZ(qWaist[0])
    .rotateY(qWaist[1]);

  tLShoulder = trcopy(tTorso)
    .translateY(shoulderOffsetY)
    .translateZ(shoulderOffsetZ)        
    .rotateY(qLArm[0])
    .rotateZ(qLArm[1])
    .rotateX(qLArm[2]);

  tLElbow = trcopy(tLShoulder)
    .translateX(upperArmLength)
    .translateZ(elbowOffsetX)
    .rotateY(qLArm[3]);
  
  tLWrist = trcopy(tLElbow)
    .translateZ(-elbowOffsetX)
    .rotateX(qLArm[4]);         
  
  tLHand = trcopy(tLWrist)
    .translateX(lowerArmLength)          
    .rotateZ(qLArm[5])
    .rotateX(qLArm[6]);


  tRShoulder = trcopy(tTorso)
    .translateY(-shoulderOffsetY)
    .translateZ(shoulderOffsetZ)        
    .rotateY(qRArm[0])
    .rotateZ(qRArm[1])
    .rotateX(qRArm[2]);

  tRElbow = trcopy(tRShoulder)
    .translateX(upperArmLength)
    .translateZ(elbowOffsetX)
    .rotateY(qRArm[3]);    

  tRWrist = trcopy(tRElbow)
    .translateZ(-elbowOffsetX)
    .rotateX(qRArm[4]);            

  tRHand = trcopy(tRWrist)
    .translateX(lowerArmLength)          
    .rotateZ(qRArm[5])
    .rotateX(qRArm[6]);













  tPelvisCOM = tPelvis
    .translateX(comPelvisX)
    .translateZ(comPelvisZ);

  tTorsoCOM = tTorso
    .translateX(comTorsoX)
    .translateZ(comTorsoZ);

  tLUArmCOM = tLShoulder
    .translateX(comUpperArmX)
    .translateZ(comUpperArmZ);
        
  tLElbowCOM = tLElbow
    .translateX(comElbowX)
    .translateZ(comElbowZ);

  tLLArmCOM = tLWrist
    .translateX(comLowerArmX);
  
  tLWristCOM = trcopy(tLHand)
    .translateX(comWristX)
    .translateZ(comWristZ);

  tLHandCOM = tLHand
    .translateX(handOffsetX)
    .translateY(-handOffsetY);



  tRUArmCOM = tRShoulder
    .translateX(comUpperArmX)
    .translateZ(comUpperArmZ);

  tRElbowCOM = tRElbow
    .translateX(comElbowX)
    .translateZ(comElbowZ);

  tRLArmCOM = tRWrist
    .translateX(comLowerArmX);

  tRWristCOM = trcopy(tRHand)
    .translateX(comWristX)
    .translateZ(comWristZ);    

  tRHandCOM = tRHand
    .translateX(handOffsetX)
    .translateY(handOffsetY);    





  r[0] = 
         mPelvis * tPelvisCOM(0,3) +
         mTorso * tTorsoCOM(0,3) +
         mUpperArm * (tLUArmCOM(0,3) + tRUArmCOM(0,3))+
         mElbow * (tLElbowCOM(0,3) + tRElbowCOM(0,3))+
         mLowerArm * (tLLArmCOM(0,3)+ tRLArmCOM(0,3))+
         mWrist * (tLWristCOM(0,3) + tRWristCOM(0,3))+
         mLHand * tLHandCOM(0,3) + 
         mRHand * tRHandCOM(0,3);

  r[1] = mPelvis * tPelvisCOM(1,3) +
         mTorso * tTorsoCOM(1,3) +
         mUpperArm * (tLUArmCOM(1,3) + tRUArmCOM(1,3))+
         mElbow * (tLElbowCOM(1,3) + tRElbowCOM(1,3))+
         mLowerArm * (tLLArmCOM(1,3)+ tRLArmCOM(1,3))+
         mWrist * (tLWristCOM(1,3) + tRWristCOM(1,3))+
         mLHand * tLHandCOM(1,3) + 
         mRHand * tRHandCOM(1,3);

  r[2] = mPelvis * tPelvisCOM(2,3) +
         mTorso * tTorsoCOM(2,3) +
         mUpperArm * (tLUArmCOM(2,3) + tRUArmCOM(2,3))+
         mElbow * (tLElbowCOM(2,3) + tRElbowCOM(2,3))+
         mLowerArm * (tLLArmCOM(2,3)+ tRLArmCOM(2,3))+
         mWrist * (tLWristCOM(2,3) + tRWristCOM(2,3))+
         mLHand * tLHandCOM(2,3) + 
         mRHand * tRHandCOM(2,3);

  r[3] = mPelvis + mTorso + 2* (mUpperArm + mElbow + mLowerArm + mWrist) + mLHand + mRHand;

  return r;
}

std::vector<double>
THOROP_kinematics_com_leg(const double *q, double bodyPitch, int is_left)  
{
  /* inverse kinematics to convert joint angles to servo positions */
  std::vector<double> r(4);

  Transform tPelvis, 
            tHip, tKnee, tAnkle,
            tULegCOM, tLLegCOM, tFootCOM;
 
  tPelvis = tPelvis.rotateY(bodyPitch);

  if (is_left>0) 
    tHip = trcopy(tPelvis)
        .translateY(hipOffsetY)
        .translateZ(hipOffsetZ);
  else 
    tHip = trcopy(tPelvis)
        .translateY(-hipOffsetY)
        .translateZ(hipOffsetZ);
  
  tHip = tHip
        .rotateZ(q[0])
        .rotateX(q[1])
        .rotateY(q[2]);

  tKnee = trcopy(tHip)
        .translateX(kneeOffsetX)
        .translateZ(-thighLength)        
        .rotateY(q[3]);

  tAnkle = trcopy(tKnee)
        .translateX(-kneeOffsetX)
        .translateZ(-tibiaLength)        
        .rotateY(q[4])
        .rotateX(q[5]);

  tULegCOM = tHip
        .translateX(comUpperLegX)
        .translateY(comUpperLegY)
        .translateZ(comUpperLegZ);

  tLLegCOM = tKnee
        .translateX(comLowerLegX)
        .translateY(comLowerLegY)
        .translateZ(comLowerLegZ);

  tFootCOM = tAnkle
        .translateX(comFootX)
        .translateZ(comFootZ);

  r[0] = mUpperLeg * tULegCOM(0,3) +
        mLowerLeg * tLLegCOM(0,3) +
        mFoot * tFootCOM(0,3) ;
         
  r[1] = mUpperLeg * tULegCOM(1,3) +
        mLowerLeg * tLLegCOM(1,3) +
        mFoot * tFootCOM(1,3) ;         

  r[2] = mUpperLeg * tULegCOM(2,3) +
        mLowerLeg * tLLegCOM(2,3) +
        mFoot * tFootCOM(2,3) ;
  
  r[3] = mUpperLeg + mLowerLeg + mFoot;
  
  return r;
}

std::vector<double>
THOROP_kinematics_calculate_support_torque(
  const double *qWaist,
  const double *qLArm,
  const double *qRArm,
  const double *qLLeg,
  const double *qRLeg,
  double bodyPitch,
  int supportLeg,
  const double *uTorsoAcc,
  double mLHand,
  double mRHand
   ){

  Transform tPelvis, tHip;

  std::vector<double> com_upperbody(4);
  std::vector<double> com_left_leg(4);
  std::vector<double> com_right_leg(4);

  std::vector<double> rel_com_hip(4);
  std::vector<double> rel_com_knee(4);
  std::vector<double> rel_com_ankle(4);


  com_upperbody = THOROP_kinematics_com_upperbody(qWaist, qLArm, qRArm,bodyPitch, mLHand, mRHand);
  com_left_leg = THOROP_kinematics_com_leg(qLLeg,bodyPitch,1);
  com_right_leg = THOROP_kinematics_com_leg(qRLeg,bodyPitch,0);

  tPelvis = tPelvis.rotateY(bodyPitch);

/*
  printf("left_leg com: %.3f %.3f %.3f",
    com_left_leg[0]/com_left_leg[3],
    com_left_leg[1]/com_left_leg[3],
    com_left_leg[2]/com_left_leg[3]);
  printf("right_leg com: %.3f %.3f %.3f",
    com_right_leg[0]/com_right_leg[3],
    com_right_leg[1]/com_right_leg[3],
    com_right_leg[2]/com_right_leg[3]);
*/

  if (supportLeg==0){ //left support
    //Add the moment of the free leg to the torso
    com_upperbody[0] = com_upperbody[0] + com_right_leg[0];
    com_upperbody[1] = com_upperbody[1] + com_right_leg[1];
    com_upperbody[2] = com_upperbody[2] + com_right_leg[2];
    com_upperbody[3] = com_upperbody[3] + com_right_leg[3];    

    tHip = trcopy(tPelvis).translateY(hipOffsetY).translateZ(-hipOffsetZ);

  }else{
    //Add the moment of the free leg to the torso
    com_upperbody[0] = com_upperbody[0] + com_left_leg[0];
    com_upperbody[1] = com_upperbody[1] + com_left_leg[1];
    com_upperbody[2] = com_upperbody[2] + com_left_leg[2];
    com_upperbody[3] = com_upperbody[3] + com_left_leg[3];    

    tHip = trcopy(tPelvis).translateY(-hipOffsetY).translateZ(-hipOffsetZ);
  }

  //Get the relative COM position at the support hip joint
  rel_com_hip[0] = com_upperbody[0]/com_upperbody[3] - tHip(0,3);
  rel_com_hip[1] = com_upperbody[1]/com_upperbody[3] - tHip(1,3);
  rel_com_hip[2] = com_upperbody[2]/com_upperbody[3] - tHip(2,3);

/*
  printf("COM pos from support hip joint: %.2f %.2f %.2f",
    rel_com_hip[0],
    rel_com_hip[1],
    rel_com_hip[2]
    );
*/

  Transform tRelCOMHip;
  tRelCOMHip = tRelCOMHip
        .translateX(rel_com_hip[0])
        .translateY(rel_com_hip[1])
        .translateZ(rel_com_hip[2]);

  //Calculate the moment at the support hip joint

  std::vector<double> upperbody_force(3);

  upperbody_force[0] = uTorsoAcc[0] * com_upperbody[3];
  upperbody_force[1] = uTorsoAcc[1] * com_upperbody[3];
  upperbody_force[2] = -9.8 * com_upperbody[3];

  //Now tilt the force back

  double tilted_force_x = 
    cos(bodyPitch)*upperbody_force[0]
    -sin(bodyPitch)*upperbody_force[2];
  double tilted_force_y = upperbody_force[1];
  double tilted_force_z = 
    sin(bodyPitch)*upperbody_force[0]+
    cos(bodyPitch)*upperbody_force[2];

  double tilted_rel_com_hip_x = 
    cos(bodyPitch)*rel_com_hip[0]
    -sin(bodyPitch)*rel_com_hip[2];
  double tilted_rel_com_hip_y = rel_com_hip[1];
  double tilted_rel_com_hip_z = 
    sin(bodyPitch)*rel_com_hip[0]+
    cos(bodyPitch)*rel_com_hip[2];


  double torque_x = tilted_rel_com_hip_z * tilted_force_x  
                  + tilted_rel_com_hip_x * tilted_force_z;  

  double torque_y = tilted_rel_com_hip_z * tilted_force_y
                  + tilted_rel_com_hip_y * tilted_force_z;  

  printf("Mass torque: %.3f",+ tilted_rel_com_hip_y * tilted_force_z);
  printf("Acc torque: %.3f",+ tilted_rel_com_hip_z * tilted_force_y);


//printf("Total free mass: %.2f ",com_upperbody[3]);

  printf("Hip torque: X %.3f Y %.3f\n",torque_x,torque_y);


  std::vector<double> r(4);
  return r;
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
THOROP_kinematics_inverse_wrist(Transform trWrist, int arm, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist) {
  //calculate soulder and elbow angle given wrist POSITION
  // Shoulder yaw angle is given

  Transform t;
  //Getting rid of hand, shoulder offsets

//Forward kinematics:
/*
    .rotateY(bodyPitch)
    .rotateZ(qWaist[0]).rotateY(qWaist[1])
    .translateY(shoulderOffsetX)
    .translateY(shoulderOffsetY)
    .translateZ(shoulderOffsetZ)
    .mDH(-PI/2, 0, q[0], 0)
    .mDH(PI/2, 0, PI/2+q[1], 0)
    .mDH(PI/2, 0, PI/2+q[2], upperArmLength)
    .mDH(PI/2, elbowOffsetX, q[3], 0)
    .mDH(-PI/2, -elbowOffsetX, -PI/2+q[4], lowerArmLength);    
*/



  if (arm==ARM_LEFT){
    t=t
      .translateZ(-shoulderOffsetZ)
      .translateY(-shoulderOffsetY)
      .translateY(-shoulderOffsetX)
      .rotateY(-qWaist[1])
      .rotateZ(-qWaist[0])
      .rotateY(-bodyPitch)
      *trWrist;
  }else{
    t=t
      .translateZ(-shoulderOffsetZ)
      .translateY(shoulderOffsetY)
      .translateY(shoulderOffsetX)
      .rotateY(-qWaist[1])
      .rotateZ(-qWaist[0])
      .rotateY(-bodyPitch)
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
THOROP_kinematics_inverse_arm_7(Transform trArm, int arm, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist) 
{
  // Closed-form inverse kinematics for THOR-OP 7DOF arm
  // (pitch-roll-yaw-pitch-yaw-roll-yaw)
  // Shoulder yaw angle is given

//Forward kinematics:
/*
    .rotateY(bodyPitch)
    .rotateZ(qWaist[0]).rotateY(qWaist[1])
    .translateY(shoulderOffsetY)
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
*/


  Transform t;

  //Getting rid of hand, shoulder offsets
  if (arm==ARM_LEFT){
    t=t
    .translateZ(-shoulderOffsetZ)
  	.translateY(-shoulderOffsetY)
    .translateY(-shoulderOffsetX)
    .rotateY(-qWaist[1])
    .rotateZ(-qWaist[0])
    .rotateY(-bodyPitch)
	*trArm
    .translateZ(-handOffsetZ)
    .translateY(handOffsetY)
	  .translateX(-handOffsetX);
  }else{
    t=t
    .translateZ(-shoulderOffsetZ)
    .translateY(shoulderOffsetY)
    .translateY(shoulderOffsetX)
    .rotateY(-qWaist[1])
    .rotateZ(-qWaist[0])
    .rotateY(-bodyPitch)
	*trArm
    .translateZ(-handOffsetZ)
    .translateY(-handOffsetY)
  	.translateX(-handOffsetX);
  }

  Transform trArmRot; //Only the rotation part of trArm

  trArmRot = trArmRot
      .rotateY(-qWaist[1])
      .rotateZ(-qWaist[0])
      .rotateY(-bodyPitch)
      *trArm
     .translateZ(-trArm(2,3))
     .translateY(-trArm(1,3))
     .translateX(-trArm(0,3));     

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
THOROP_kinematics_inverse_arm_given_wrist(Transform trArm, const double *qOrg, double bodyPitch, const double *qWaist) 
{
  //Calculate the wrist angle given the wrist position and the target transform 

printf("qWaist: %f %f\n",qWaist[0],qWaist[1]);
printf("bodyPitch: %f \n",bodyPitch);

  Transform trArmRot; //Only the rotation part of trArm

  trArmRot = trArmRot
      .rotateY(-qWaist[1])
      .rotateZ(-qWaist[0])
      .rotateY(-bodyPitch)
      *trArm
     .translateZ(-trArm(2,3))
     .translateY(-trArm(1,3))
     .translateX(-trArm(0,3));     

  //Pelvis to wrist transform  
  Transform tInvRot;
  tInvRot = tInvRot
  .rotateY(-qOrg[3])
  .rotateX(-qOrg[2])
  .rotateZ(-qOrg[1])
  .rotateY(-qOrg[0]);
  


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
  qArm[0] = qOrg[0];
  qArm[1] = qOrg[1];
  qArm[2] = qOrg[2];
  qArm[3] = qOrg[3];

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
THOROP_kinematics_inverse_l_arm_7(Transform trArm, const double *qOrg, double shoulderYaw , double bodyPitch, const double *qWaist) 
{
  return THOROP_kinematics_inverse_arm_7(trArm, ARM_LEFT, qOrg, shoulderYaw, bodyPitch, qWaist);
}

  std::vector<double>
THOROP_kinematics_inverse_r_arm_7(Transform trArm, const double *qOrg,double shoulderYaw, double bodyPitch, const double *qWaist) 
{
  return THOROP_kinematics_inverse_arm_7(trArm, ARM_RIGHT, qOrg, shoulderYaw, bodyPitch, qWaist);
}

  std::vector<double>
THOROP_kinematics_inverse_l_wrist(Transform trWrist, const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist) 
{
  return THOROP_kinematics_inverse_wrist(trWrist, ARM_LEFT, qOrg, shoulderYaw, bodyPitch, qWaist);
}

  std::vector<double>
THOROP_kinematics_inverse_r_wrist(Transform trWrist,const double *qOrg, double shoulderYaw, double bodyPitch, const double *qWaist) 
{
  return THOROP_kinematics_inverse_wrist(trWrist, ARM_RIGHT, qOrg, shoulderYaw, bodyPitch, qWaist);
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
