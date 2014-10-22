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
THOROP_kinematics_forward_l_arm_7(const double *q, double bodyPitch, const double *qWaist, 
  double handOffsetXNew, double handOffsetYNew, double handOffsetZNew)
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
    .translateX(handOffsetXNew)
    .translateY(-handOffsetYNew)
    .translateZ(handOffsetZNew);    

  return t;
}


  Transform
THOROP_kinematics_forward_r_arm_7(const double *q, double bodyPitch, const double *qWaist,
   double handOffsetXNew, double handOffsetYNew, double handOffsetZNew) 
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
    .translateX(handOffsetXNew)
    .translateY(handOffsetYNew)
    .translateZ(handOffsetZNew);
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

//Calculate the point COM positions for the robot
// 6 mass for legs
// 4 mass for arms
// 1 mass for torso/pelvis

std::vector<double>
THOROP_kinematics_calculate_com_positions(
    const double *qWaist,
    const double *qLArm,
    const double *qRArm,
    const double *qLLeg,
    const double *qRLeg,

    double bodyPitch){
  
  
  Transform 
    tPelvis, tTorso, 
    tLShoulder, tLElbow, tLWrist, tLHand,
    tRShoulder, tRElbow, tRWrist, tRHand,
    tLHip, tLKnee, tLAnkle,
    tRHip, tRKnee, tRAnkle,
   
    tPelvisCOM, tTorsoCOM,
    tLUArmCOM, tLElbowCOM, tLLArmCOM, tLWristCOM, tLHandCOM,
    tRUArmCOM, tRElbowCOM, tRLArmCOM, tRWristCOM, tRHandCOM,
    tLULegCOM, tLLLegCOM, tLFootCOM,
    tRULegCOM, tRLLegCOM, tRFootCOM;
    

  tPelvis = tPelvis
    .rotateY(bodyPitch);

  tTorso = trcopy(tPelvis)
    .rotateZ(qWaist[0])
    .rotateY(qWaist[1]);

  /////////////////////////////////
  //Arm joint poistions
  /////////////////////////////////

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

  /////////////////////////////////
  //Leg joint poistions
  /////////////////////////////////


  tLHip = trcopy(tPelvis)
        .translateY(hipOffsetY)
        .translateZ(-hipOffsetZ)
        .rotateZ(qLLeg[0])
        .rotateX(qLLeg[1])
        .rotateY(qLLeg[2]);

  tLKnee = trcopy(tLHip)
        .translateX(kneeOffsetX)
        .translateZ(-thighLength)        
        .rotateY(qLLeg[3]);

  tLAnkle = trcopy(tLKnee)
        .translateX(-kneeOffsetX)
        .translateZ(-tibiaLength)        
        .rotateY(qLLeg[4])
        .rotateX(qLLeg[5]);

  tRHip = trcopy(tPelvis)
        .translateY(-hipOffsetY)
        .translateZ(-hipOffsetZ)
        .rotateZ(qRLeg[0])
        .rotateX(qRLeg[1])
        .rotateY(qRLeg[2]);

  tRKnee = trcopy(tRHip)
        .translateX(kneeOffsetX)
        .translateZ(-thighLength)        
        .rotateY(qRLeg[3]);

  tRAnkle = trcopy(tRKnee)
        .translateX(-kneeOffsetX)
        .translateZ(-tibiaLength)        
        .rotateY(qRLeg[4])
        .rotateX(qRLeg[5]);


  /////////////////////////////////
  //Body COM positions
  /////////////////////////////////

  tPelvisCOM = tPelvis
    .translateX(comPelvisX)
    .translateZ(comPelvisZ);

  tTorsoCOM = tTorso
    .translateX(comTorsoX)
    .translateZ(comTorsoZ);

  /////////////////////////////////
  //Arm COM poistions
  /////////////////////////////////

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

  /////////////////////////////////
  //Leg COM poistions
  /////////////////////////////////

  tLULegCOM = tLHip
        .translateX(comUpperLegX)
        .translateY(comUpperLegY)
        .translateZ(comUpperLegZ);

  tLLLegCOM = tLKnee
        .translateX(comLowerLegX)
        .translateY(comLowerLegY)
        .translateZ(comLowerLegZ);

  tLFootCOM = tLAnkle
        .translateX(comFootX)
        .translateZ(comFootZ);

  tRULegCOM = tRHip
        .translateX(comUpperLegX)
        .translateY(-comUpperLegY)
        .translateZ(comUpperLegZ);        

  tRLLegCOM = tRKnee
        .translateX(comLowerLegX)
        .translateY(-comLowerLegY)
        .translateZ(comLowerLegZ);

  tRFootCOM = tRAnkle
        .translateX(comFootX)
        .translateZ(comFootZ);


  //Write to a single vector

  //Mass points
  //4+4 for arms
  //6+6 for legs
  //1+1 for body

  std::vector<double> comxyz(66);

  //X coordinates
  comxyz[0] = tRULegCOM(0,3);
  comxyz[1] = tRLLegCOM(0,3);
  comxyz[2] = tRFootCOM(0,3);

  comxyz[6] = tLULegCOM(0,3);
  comxyz[7] = tLLLegCOM(0,3);
  comxyz[8] = tLFootCOM(0,3);

  comxyz[12] = tRUArmCOM(0,3);
  comxyz[13] = tRElbowCOM(0,3);
  comxyz[14] = tRLArmCOM(0,3);
  comxyz[15] = tRWristCOM(0,3);

  comxyz[16] = tLUArmCOM(0,3);
  comxyz[17] = tLElbowCOM(0,3);
  comxyz[18] = tLLArmCOM(0,3);
  comxyz[19] = tLWristCOM(0,3);

  comxyz[20] = tTorsoCOM(0,3);
  comxyz[21] = tPelvisCOM(0,3);


  //Y coordinates
  comxyz[0+22] = tRULegCOM(1,3);
  comxyz[1+22] = tRLLegCOM(1,3);
  comxyz[2+22] = tRFootCOM(1,3);

  comxyz[6+22] = tLULegCOM(1,3);
  comxyz[7+22] = tLLLegCOM(1,3);
  comxyz[8+22] = tLFootCOM(1,3);

  comxyz[12+22] = tRUArmCOM(1,3);
  comxyz[13+22] = tRElbowCOM(1,3);
  comxyz[14+22] = tRLArmCOM(1,3);
  comxyz[15+22] = tRWristCOM(1,3);

  comxyz[16+22] = tLUArmCOM(1,3);
  comxyz[17+22] = tLElbowCOM(1,3);
  comxyz[18+22] = tLLArmCOM(1,3);
  comxyz[19+22] = tLWristCOM(1,3);

  comxyz[20+22] = tTorsoCOM(1,3);
  comxyz[21+22] = tPelvisCOM(1,3);

  //Z coordinates
  comxyz[0+44] = tRULegCOM(2,3);
  comxyz[1+44] = tRLLegCOM(2,3);
  comxyz[2+44] = tRFootCOM(2,3);

  comxyz[6+44] = tLULegCOM(2,3);
  comxyz[7+44] = tLLLegCOM(2,3);
  comxyz[8+44] = tLFootCOM(2,3);

  comxyz[12+44] = tRUArmCOM(2,3);
  comxyz[13+44] = tRElbowCOM(2,3);
  comxyz[14+44] = tRLArmCOM(2,3);
  comxyz[15+44] = tRWristCOM(2,3);

  comxyz[16+44] = tLUArmCOM(2,3);
  comxyz[17+44] = tLElbowCOM(2,3);
  comxyz[18+44] = tLLArmCOM(2,3);
  comxyz[19+44] = tLWristCOM(2,3);

  comxyz[20+44] = tTorsoCOM(2,3);
  comxyz[21+44] = tPelvisCOM(2,3);

  return comxyz;
}




//Calculate the point COM positions for the robot
// 6 mass for legs
// 4 mass for arms
// 1 mass for torso/pelvis

//This calculates GLOBAL com positions of mass particles
//Assuming the support foot is on the ground

std::vector<double>
THOROP_kinematics_calculate_com_positions_global(
    const double *qWaist,
    const double *qLArm,
    const double *qRArm,
    const double *qLLeg,
    const double *qRLeg,

    const double *uSupport,
    int supportLeg){
  
  Transform 
    tSupportFoot,
    tPelvis, tTorso, 
    tLShoulder, tLElbow, tLWrist, tLHand,
    tRShoulder, tRElbow, tRWrist, tRHand,
    tLHip, tLKnee, tLAnkle,
    tRHip, tRKnee, tRAnkle,
   
    tPelvisCOM, tTorsoCOM,
    tLUArmCOM, tLElbowCOM, tLLArmCOM, tLWristCOM, tLHandCOM,
    tRUArmCOM, tRElbowCOM, tRLArmCOM, tRWristCOM, tRHandCOM,
    tLULegCOM, tLLLegCOM, tLFootCOM,
    tRULegCOM, tRLLegCOM, tRFootCOM;
    
  tSupportFoot = tSupportFoot
    .translateX(uSupport[0])
    .translateY(uSupport[1])
    .translateZ(footHeight)
    .rotateZ(uSupport[2]);

  if (supportLeg==0){ //Left support
    //Reverse calculate up to pelvis
    tLAnkle = trcopy(tSupportFoot);
    tLKnee = trcopy(tLAnkle)
      .rotateX(-qLLeg[5])
      .rotateY(-qLLeg[4])
      .translateZ(tibiaLength)
      .translateX(kneeOffsetX);

    tLHip = trcopy(tLKnee)
      .rotateY(-qLLeg[3])
      .translateZ(thighLength)
      .translateX(-kneeOffsetX);

    tPelvis = trcopy(tLHip)
      .rotateY(-qLLeg[2])
      .rotateX(-qLLeg[1])
      .rotateZ(-qLLeg[0])
      .translateZ(hipOffsetZ)
      .translateY(-hipOffsetY);

    //Forward calculaet right leg

    tRHip = trcopy(tPelvis)
        .translateY(-hipOffsetY)
        .translateZ(-hipOffsetZ)
        .rotateZ(qRLeg[0])
        .rotateX(qRLeg[1])
        .rotateY(qRLeg[2]);

    tRKnee = trcopy(tRHip)
        .translateX(kneeOffsetX)
        .translateZ(-thighLength)        
        .rotateY(qRLeg[3]);

    tRAnkle = trcopy(tRKnee)
        .translateX(-kneeOffsetX)
        .translateZ(-tibiaLength)        
        .rotateY(qRLeg[4])
        .rotateX(qRLeg[5]);
  }else{
    //Reverse calculate up to pelvis
    tRAnkle = trcopy(tSupportFoot);
    tRKnee = trcopy(tRAnkle)
      .rotateX(-qRLeg[5])
      .rotateY(-qRLeg[4])
      .translateZ(tibiaLength)
      .translateX(kneeOffsetX);

    tRHip = trcopy(tRKnee)
      .rotateY(-qRLeg[3])
      .translateZ(thighLength)
      .translateX(-kneeOffsetX);

    tPelvis = trcopy(tRHip)
      .rotateY(-qRLeg[2])
      .rotateX(-qRLeg[1])
      .rotateZ(-qRLeg[0])
      .translateZ(hipOffsetZ)
      .translateY(hipOffsetY);

      //Forward calculate left leg 
    tLHip = trcopy(tPelvis)
        .translateY(hipOffsetY)
        .translateZ(-hipOffsetZ)
        .rotateZ(qLLeg[0])
        .rotateX(qLLeg[1])
        .rotateY(qLLeg[2]);

    tLKnee = trcopy(tLHip)
        .translateX(kneeOffsetX)
        .translateZ(-thighLength)        
        .rotateY(qLLeg[3]);

    tLAnkle = trcopy(tLKnee)
        .translateX(-kneeOffsetX)
        .translateZ(-tibiaLength)        
        .rotateY(qLLeg[4])
        .rotateX(qLLeg[5]);

  }

  tTorso = trcopy(tPelvis)
    .rotateZ(qWaist[0])
    .rotateY(qWaist[1]);

  /////////////////////////////////
  //Arm joint poistions
  /////////////////////////////////

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

  /////////////////////////////////
  //Body COM positions
  /////////////////////////////////

  tPelvisCOM = tPelvis
    .translateX(comPelvisX)
    .translateZ(comPelvisZ);

  tTorsoCOM = tTorso
    .translateX(comTorsoX)
    .translateZ(comTorsoZ);

  /////////////////////////////////
  //Arm COM poistions
  /////////////////////////////////

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

  /////////////////////////////////
  //Leg COM poistions
  /////////////////////////////////

  tLULegCOM = tLHip
        .translateX(comUpperLegX)
        .translateY(comUpperLegY)
        .translateZ(comUpperLegZ);

  tLLLegCOM = tLKnee
        .translateX(comLowerLegX)
        .translateY(comLowerLegY)
        .translateZ(comLowerLegZ);

  tLFootCOM = tLAnkle
        .translateX(comFootX)
        .translateZ(comFootZ);

  tRULegCOM = tRHip
        .translateX(comUpperLegX)
        .translateY(-comUpperLegY)
        .translateZ(comUpperLegZ);        

  tRLLegCOM = tRKnee
        .translateX(comLowerLegX)
        .translateY(-comLowerLegY)
        .translateZ(comLowerLegZ);

  tRFootCOM = tRAnkle
        .translateX(comFootX)
        .translateZ(comFootZ);


  //Write to a single vector

  //Mass points
  //4+4 for arms
  //6+6 for legs
  //1+1 for body

  std::vector<double> comxyz(66);

  //X coordinates
  comxyz[0] = tRULegCOM(0,3);
  comxyz[1] = tRLLegCOM(0,3);
  comxyz[2] = tRFootCOM(0,3);

  comxyz[6] = tLULegCOM(0,3);
  comxyz[7] = tLLLegCOM(0,3);
  comxyz[8] = tLFootCOM(0,3);

  comxyz[12] = tRUArmCOM(0,3);
  comxyz[13] = tRElbowCOM(0,3);
  comxyz[14] = tRLArmCOM(0,3);
  comxyz[15] = tRWristCOM(0,3);

  comxyz[16] = tLUArmCOM(0,3);
  comxyz[17] = tLElbowCOM(0,3);
  comxyz[18] = tLLArmCOM(0,3);
  comxyz[19] = tLWristCOM(0,3);

  comxyz[20] = tTorsoCOM(0,3);
  comxyz[21] = tPelvisCOM(0,3);


  //Y coordinates
  comxyz[0+22] = tRULegCOM(1,3);
  comxyz[1+22] = tRLLegCOM(1,3);
  comxyz[2+22] = tRFootCOM(1,3);

  comxyz[6+22] = tLULegCOM(1,3);
  comxyz[7+22] = tLLLegCOM(1,3);
  comxyz[8+22] = tLFootCOM(1,3);

  comxyz[12+22] = tRUArmCOM(1,3);
  comxyz[13+22] = tRElbowCOM(1,3);
  comxyz[14+22] = tRLArmCOM(1,3);
  comxyz[15+22] = tRWristCOM(1,3);

  comxyz[16+22] = tLUArmCOM(1,3);
  comxyz[17+22] = tLElbowCOM(1,3);
  comxyz[18+22] = tLLArmCOM(1,3);
  comxyz[19+22] = tLWristCOM(1,3);

  comxyz[20+22] = tTorsoCOM(1,3);
  comxyz[21+22] = tPelvisCOM(1,3);

  //Z coordinates
  comxyz[0+44] = tRULegCOM(2,3);
  comxyz[1+44] = tRLLegCOM(2,3);
  comxyz[2+44] = tRFootCOM(2,3);

  comxyz[6+44] = tLULegCOM(2,3);
  comxyz[7+44] = tLLLegCOM(2,3);
  comxyz[8+44] = tLFootCOM(2,3);

  comxyz[12+44] = tRUArmCOM(2,3);
  comxyz[13+44] = tRElbowCOM(2,3);
  comxyz[14+44] = tRLArmCOM(2,3);
  comxyz[15+44] = tRWristCOM(2,3);

  comxyz[16+44] = tLUArmCOM(2,3);
  comxyz[17+44] = tLElbowCOM(2,3);
  comxyz[18+44] = tLLArmCOM(2,3);
  comxyz[19+44] = tLWristCOM(2,3);

  comxyz[20+44] = tTorsoCOM(2,3);
  comxyz[21+44] = tPelvisCOM(2,3);

  return comxyz;
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
THOROP_kinematics_com_upperbody_2(
  const double *com, double mLHand, double mRHand){
  std::vector<double> r(4);

  int i;  
  //for (i=0;i<22;i++) {
  for (i=11;i<22;i++) {
    r[0]+= Mass[i]*com[i];
    r[1]+= Mass[i]*com[i+22];
    r[2]+= Mass[i]*com[i+44];
    r[3]+= Mass[i];
  }
  //LHAND
  r[0]+=mLHand*com[13]; //LHAND
  r[1]+=mLHand*com[13+22]; //LHAND
  r[2]+=mLHand*com[13+44]; //LHAND
  r[3]+=mLHand; 
  //RHAND
  r[0]+=mRHand*com[17]; //LHAND
  r[1]+=mRHand*com[17+22]; //LHAND
  r[2]+=mRHand*com[17+44]; //LHAND
  r[3]+=mRHand; 
  return r;
}





std::vector<double>
THOROP_kinematics_calculate_zmp(
  const double *com0, const double *com1, const double *com2,
  double dt0, double dt1){

  //todo: incorporate mLHand and mRHand
  std::vector<double> zmp(3);
  int i;
  //sum(m_i * g * (x_i - p)) = sum (m_i*z_i * x_i'')  
  // p = sum( m_i ( z_i''/g + x_i - z_i*x_i''/g ))  / sum(m_i)

  for (i=0;i<22;i++){
    double vx0 = (com1[i]-com0[i])/dt0;
    double vy0 = (com1[i+22]-com0[i+22])/dt0;
    double vz0 = (com1[i+44]-com0[i+44])/dt0;

    double vx1 = (com2[i]-com1[i])/dt1;
    double vy1 = (com2[i+22]-com1[i+22])/dt1;
    double vz1 = (com2[i+44]-com1[i+44])/dt1;

    //Calculate discrete accelleration
    double ax = (vx1-vx0)/dt1;
    double ay = (vy1-vy0)/dt1;
    double az = (vz1-vz0)/dt1;

    ax=0.0;  ay=0.0; //hack

    //Accummulate torque
    zmp[0]+= Mass[i]*(az/9.8 + com2[i] - com2[i+44] * ax/9.8 );
    zmp[1]+= Mass[i]*(az/9.8 + com2[i+22] - com2[i+44] * ay / 9.8 );
    zmp[2]+= Mass[i];
  }

  return zmp;
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
  //calculate shoulder and elbow angle given wrist POSITION
  // Shoulder yaw angle is given

  Transform t;
  //Getting rid of hand, shoulder offsets
  if (arm==ARM_LEFT){
    t=t
      .translateZ(-shoulderOffsetZ)
      .translateY(-shoulderOffsetY)
      .translateX(-shoulderOffsetX)
      .rotateY(-qWaist[1])
      .rotateZ(-qWaist[0])
      .rotateY(-bodyPitch)
      *trWrist;
  }else{
    t=t
      .translateZ(-shoulderOffsetZ)
      .translateY(shoulderOffsetY)
      .translateX(shoulderOffsetX)
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
THOROP_kinematics_inverse_arm_7(Transform trArm, int arm, const double *qOrg, double shoulderYaw, 
  double bodyPitch, const double *qWaist, double handOffsetXNew, double handOffsetYNew, double handOffsetZNew, int flip_shoulderroll) 
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
    .translateX(-shoulderOffsetX)
    .rotateY(-qWaist[1])
    .rotateZ(-qWaist[0])
    .rotateY(-bodyPitch)
	*trArm
    .translateZ(-handOffsetZNew)
    .translateY(handOffsetYNew)
	  .translateX(-handOffsetXNew);
  }else{
    t=t
    .translateZ(-shoulderOffsetZ)
    .translateY(shoulderOffsetY)
    .translateX(shoulderOffsetX)
    .rotateY(-qWaist[1])
    .rotateZ(-qWaist[0])
    .rotateY(-bodyPitch)
	*trArm
    .translateZ(-handOffsetZNew)
    .translateY(-handOffsetYNew)
  	.translateX(-handOffsetXNew);
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

  
  double a,b,c;
  double shoulderPitch, shoulderRoll;
  double err1,err2;

//Solve for sin (roll limited to -pi/2 to pi/2)
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

  //Now we extend shoulder roll angle to 0~pi for left, -pi~0 for right
  //We manually choose either shoulderRoll area (normal / flipped up)
  if (flip_shoulderroll>0){
    if (arm==ARM_LEFT) shoulderRoll = PI-shoulderRoll;
    else shoulderRoll = (-PI) - shoulderRoll;
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

//printf("qWaist: %f %f\n",qWaist[0],qWaist[1]);
//printf("bodyPitch: %f \n",bodyPitch);

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
THOROP_kinematics_inverse_l_arm_7(Transform trArm, const double *qOrg, double shoulderYaw , double bodyPitch, const double *qWaist,
    double handOffsetXNew, double handOffsetYNew, double handOffsetZNew, int flip_shoulderroll) 
{
  return THOROP_kinematics_inverse_arm_7(trArm, ARM_LEFT, qOrg, shoulderYaw, bodyPitch, qWaist, handOffsetXNew, handOffsetYNew, handOffsetZNew, flip_shoulderroll);
}

  std::vector<double>
THOROP_kinematics_inverse_r_arm_7(Transform trArm, const double *qOrg,double shoulderYaw, double bodyPitch, const double *qWaist,
   double handOffsetXNew, double handOffsetYNew, double handOffsetZNew, int flip_shoulderroll)  
{
  return THOROP_kinematics_inverse_arm_7(trArm, ARM_RIGHT, qOrg, shoulderYaw, bodyPitch, qWaist, handOffsetXNew, handOffsetYNew, handOffsetZNew, flip_shoulderroll);
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

/*
std::vector<double>
THOROP_kinematics_inverse_leg(Transform trLeg, int leg)
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
  xLeg[2] -= footHeight;

  // Knee pitch
  double dLeg = xLeg[0]*xLeg[0] + xLeg[1]*xLeg[1] + xLeg[2]*xLeg[2];
  double cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);

  //Automatic heel lift when IK limit is reached
  double footCompZ = 0;
  double ankle_tilt_angle = 0;

  if ((cKnee>1) || (cKnee<-1)) {
    double f_h = 0;
    printf("leg %d cos knee: %.2f\n",leg,cKnee);
    f_h = sqrt((dTibia+dThigh)*(dTibia+dThigh) - xLeg[0]*xLeg[0] - xLeg[1]*xLeg[1]);
    printf("Needed z offset:%.2f\n", xLeg[2]-f_h);

    footCompZ = xLeg[2]-f_h;
    xLeg[2] = f_h;

    double angle1 = atan2(footHeight,footToeX);
    double dist1 = sqrt(footToeX*footToeX + footHeight*footHeight);

    //sin (angle1+da)*dist = footSizeZ + footCompZ

    double val1 = (footHeight+footCompZ)/dist1;
    printf("asin value:%.2f\n",val1);

    if (val1>1) val1= 1;
    if (val1<-1) val1= -1;

    ankle_tilt_angle = asin( val1 ) - angle1;
    printf("Ankle tilt angle: %.2f\n",ankle_tilt_angle*180/3.1415);

    if (ankle_tilt_angle>30*3.1415/180)  ankle_tilt_angle=30*3.1415/180;

    xLeg[0] = xLeg[0] - 
      (sin(ankle_tilt_angle)*footHeight + (1-cos(ankle_tilt_angle))*footToeX);

    dLeg = xLeg[0]*xLeg[0] + xLeg[1]*xLeg[1] + xLeg[2]*xLeg[2];
    cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);
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

*/


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
THOROP_kinematics_inverse_leg(Transform trLeg, int leg)
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
  double dLegMax = dTibia + dThigh;

  double cKnee = .5*(dLeg-dTibia*dTibia-dThigh*dThigh)/(dTibia*dThigh);

  //Automatic heel lift when IK limit is reached
  double footCompZ = 0;
  double ankle_tilt_angle = 0;

  double footC = sqrt(footHeight*footHeight + footToeX*footToeX);
  double afootA = asin(footHeight/footC);
  double xLeg0Mod = xLeg[0] - footToeX;

  double footHeelC = sqrt(footHeight*footHeight + footHeelX*footHeelX);
  double afootHeel = asin(footHeight/footC);
  double xLeg0ModHeel = xLeg[0] + footHeelX;

  if (dLeg>dLegMax*dLegMax) {
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

    if (d > 0){
      double a1 = (-b + sqrt(d))/2/a;
      double a2 = (-b - sqrt(d))/2/a;
      double err1 = fabs(p*a1 + q*sqrt(1-a1*a1)+r);
      double err2 = fabs(p*a2 + q*sqrt(1-a2*a2)+r);
      double ankle_tilt_angle1 = asin(a1)-afootA;
      double ankle_tilt_angle2 = asin(a2)-afootA;

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

//      printf("errors:%.8f , %.8f\n",err1,err2);
//      printf("Heel lift angle: %.2f\n",ankle_tilt_angle*180/3.1415);
    }else {
//      printf("SOLUTION ERROR!!!!");
      ankle_tilt_angle = 0;
    }
    if (ankle_tilt_angle>30*3.1415/180)  ankle_tilt_angle=30*3.1415/180;


/*

//---------------------------------------------------------------------------------------------------  
    //Calculate the amount of toe lift
    
    // then rotated ankle position (ax,az) is footToeX-cos(a+aFootA)*footC, sin(a+aFootA)*footC
    // or footToeX-cosb*footC, sinb *footC
    // then 
    // (xLeg[0]-ax)^2 + xLeg[1]^2 + (xLeg[2]-az)^2 = dLegMax^2
    // or (xLeg0Mod + cosb*footC)^2 + xLeg[1]^2 + (xLeg[2]-sinb*footC)^2 = dLegMax^2
    //this eq: p * sinb + q*cosb + r = 0

    double p = -2*footHeelC*xLeg[2];
    double q = -2*footHeelC*xLeg0ModHeel;
    double r = xLeg0ModHeel*xLeg0ModHeel + xLeg[1]*xLeg[1] +xLeg[2]*xLeg[2] 
                - dLegMax*dLegMax +footHeelC*footHeelC;
    double a = (p*p/q/q + 1);
    double b = 2*p*r/q/q;
    double c = r*r/q/q - 1; 
    double d = b*b-4*a*c;

    if (d > 0){
      double a1 = (-b + sqrt(d))/2/a;
      double a2 = (-b - sqrt(d))/2/a;
      double err1 = fabs(p*a1 + q*sqrt(1-a1*a1)+r);
      double err2 = fabs(p*a2 + q*sqrt(1-a2*a2)+r);
      double ankle_tilt_angle1 = - (asin(a1)-afootHeel);
      double ankle_tilt_angle2 = - (asin(a2)-afootHeel);
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
//      printf("heel-ankle angle :%.3f\n",afootHeel*180/3.1415);
//      printf("sinb values: %.3f,%.3f\n",a1,a2);

      printf("angle values: %.3f,%.3f\n",asin(a1)*180/3.14,asin(a2)*180/3.14);
      printf("errors:%.8f , %.8f\n",err1,err2);
      printf("Toe lift angle: %.2f\n",-ankle_tilt_angle*180/3.1415);
    }else {
      printf("SOLUTION ERROR!!!!");
      ankle_tilt_angle = 0;
    }
//---------------------------------------------------------------------------------------------------  

*/



    //Compensate the ankle position according to ankle tilt angle
    xLeg[0] = xLeg[0] - (sin(ankle_tilt_angle)*footHeight + (1-cos(ankle_tilt_angle))*footToeX);
    xLeg[2] = xLeg[2] - sin(afootA+ankle_tilt_angle)*footC;
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
THOROP_kinematics_inverse_l_leg(Transform trLeg)
{return THOROP_kinematics_inverse_leg(trLeg, LEG_LEFT);}

  std::vector<double>
THOROP_kinematics_inverse_r_leg(Transform trLeg)
{return THOROP_kinematics_inverse_leg(trLeg, LEG_RIGHT);}




		/*
	std::vector<double> THOROP_kinematics_inverse_arm(Transform trArm, int arm, const double *qOrg, double shoulderYaw, 
	  double bodyPitch, const double *qWaist, double handOffsetXNew, double handOffsetYNew, double handOffsetZNew, int flip_shoulderroll) 
	{
		*/
	/* NOTE: !!Assume the ARM_LEFT/ARM_RIGHT only!!! */
	// TODO: Add flip_shoulderroll
	// TODO: Add end effector OffsetNew code back in
	// TODO: Deal with the waist and bodyPitch
	// NOTE: trArm *WILL* be modified
std::vector<double> THOROP_kinematics_inverse_arm(Transform trArm, std::vector<double>& qOrg, double shoulderYaw) {
  // Closed-form inverse kinematics for THOR-OP 7DOF arm
  // (pitch-roll-yaw-pitch-yaw-roll-yaw)
  // Shoulder yaw angle is given
	
	//printf("shoulderYaw: %f\n", shoulderYaw);

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


	// t is trArm with no offsets of waist roations or shoulder position
	Transform t;
	t = t * trArm;
	/*
	Transform t;
	// Left
//  t = t
//	  .translateZ(-shoulderOffsetZ)
//		.translateY(-shoulderOffsetY)
//	  .translateX(-shoulderOffsetX) * trArm;

	// Right
  t = t
	  .translateZ(-shoulderOffsetZ)
		.translateY(shoulderOffsetY)
	  .translateX(-shoulderOffsetX) * trArm;
	*/
		
	// trArmRot is the Rotation-only part of trArm
	// NOTE: Here, trArm is mutated
	Transform trArmRot;
	trArmRot = trArmRot * trArm
		.translateZ(-trArm(2,3))
		.translateY(-trArm(1,3))
		.translateX(-trArm(0,3));

	// TODO: Add the offsets back in
	/*
	Transform t;
  // Getting rid of hand, shoulder offsets
  if (arm==ARM_LEFT){
    t=t
    .translateZ(-shoulderOffsetZ)
  	.translateY(-shoulderOffsetY)
    .translateY(-shoulderOffsetX)
    .rotateY(-qWaist[1])
    .rotateZ(-qWaist[0])
    .rotateY(-bodyPitch)
	*trArm
    .translateZ(-handOffsetZNew)
    .translateY(handOffsetYNew)
	  .translateX(-handOffsetXNew);
  }else{
    t=t
    .translateZ(-shoulderOffsetZ)
    .translateY(shoulderOffsetY)
    .translateY(shoulderOffsetX)
    .rotateY(-qWaist[1])
    .rotateZ(-qWaist[0])
    .rotateY(-bodyPitch)
	*trArm
    .translateZ(-handOffsetZNew)
    .translateY(-handOffsetYNew)
  	.translateX(-handOffsetXNew);
  }
  trArmRot = trArmRot
      .rotateY(-qWaist[1])
      .rotateZ(-qWaist[0])
      .rotateY(-bodyPitch)
      *trArm
     .translateZ(-trArm(2,3))
     .translateY(-trArm(1,3))
     .translateX(-trArm(0,3)); 
	*/    

//---------------------------------------------------------------------------
// Calculating elbow pitch from shoulder-wrist distance
//---------------------------------------------------------------------------

  double xWrist[3];
  for (int i = 0; i < 3; i++) xWrist[i]=0;
  t.apply(xWrist);

  double dWrist = xWrist[0]*xWrist[0]+xWrist[1]*xWrist[1]+xWrist[2]*xWrist[2];
  double cElbow = .5*(dWrist-dUpperArm*dUpperArm-dLowerArm*dLowerArm)/(dUpperArm*dLowerArm);
  //if (cElbow > 1) cElbow = 1; else if (cElbow < -1) cElbow = -1;
	cElbow = cElbow > .999 ? 1 : (cElbow < -.999 ? -1 : 0);

  // SJ: Robot can have TWO elbow pitch values (near elbowPitch==0)
  // We are only using the smaller one (arm more bent) 
  double elbowPitch = -acos(cElbow) - aUpperArm - aLowerArm;  

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

  
  double a,b,c;
  double shoulderPitch, shoulderRoll;
  double err1,err2;

	// Solve for sin (roll limited to -pi/2 to pi/2)
  a = m(0,3)*m(0,3) + m(1,3)*m(1,3);
  b = -m(0,3)*xWrist[1];
  c = xWrist[1]*xWrist[1] - m(1,3)*m(1,3);
	
	// NaN handling
  if ((b*b-a*c<0)|| a==0 ) {
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
    if (err1*err1<err2*err2){
			shoulderRoll = shoulderRoll1;
		}
    else {
			shoulderRoll = shoulderRoll2;
		}
  }

  //Now we extend shoulder roll angle to 0~pi for left, -pi~0 for right
  //We manually choose either shoulderRoll area (normal / flipped up)
	// TODO: Add this back in
	/*
  if (flip_shoulderroll>0){
    if (arm==ARM_LEFT) shoulderRoll = PI-shoulderRoll;
    else shoulderRoll = (-PI) - shoulderRoll;
  }
	*/

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

  shoulderPitch = atan2(s1, c1);

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

  // Singular point: just use current angles
  if(swa < 0.00001){
    wristYaw_a = qOrg[4];    
    wristRoll_a = qOrg[5];
    wristYaw2_a = qOrg[6];
    wristYaw_b = qOrg[4];
    wristRoll_b = qOrg[5];
    wristYaw2_b = qOrg[6];
  } 

  // Select the closest solution to current joint angle  
  bool select_a = false;
  double err_a = fmodf(qOrg[4] - wristYaw_a+5*PI, 2*PI) - PI;
  double err_b = fmodf(qOrg[4] - wristYaw_b+5*PI, 2*PI) - PI;
  if (err_a*err_a<err_b*err_b){
		select_a = true;
	}
  
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
THOROP_kinematics_inverse_wrist(Transform trWrist, std::vector<double>& qOrg, double shoulderYaw /*, double bodyPitch, const double *qWaist */) {
  //calculate shoulder and elbow angle given wrist POSITION
  // Shoulder yaw angle is given

  Transform t;
	t = t*trWrist;

//---------------------------------------------------------------------------
// Calculating elbow pitch from shoulder-wrist distance
//---------------------------------------------------------------------------
     
  double xWrist[3] = {0,0,0};
  //for (int i = 0; i < 3; i++) xWrist[i]=0;
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
    if (err1*err1<err2*err2) {
			shoulderRoll = shoulderRoll1;
		}
    else {
			shoulderRoll = shoulderRoll2;
		}
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
