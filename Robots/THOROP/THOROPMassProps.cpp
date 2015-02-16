#include "THOROPKinematics.h"
/* 7 DOF */

enum {LEG_LEFT = 0, LEG_RIGHT = 1};
enum {ARM_LEFT = 0, ARM_RIGHT = 1};

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

    double mLHand,
    double mRHand,
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





//make a single compound COM position (from pelvis frame)
  std::vector<double> r(4);


 r[0] = 
         mPelvis * tPelvisCOM(0,3) +
         mTorso * tTorsoCOM(0,3) +
         mUpperArm * (tLUArmCOM(0,3) + tRUArmCOM(0,3))+
         mElbow * (tLElbowCOM(0,3) + tRElbowCOM(0,3))+
         mLowerArm * (tLLArmCOM(0,3)+ tRLArmCOM(0,3))+
         mWrist * (tLWristCOM(0,3) + tRWristCOM(0,3))+
         mLHand * tLHandCOM(0,3) + 
         mRHand * tRHandCOM(0,3) +

         mUpperLeg * (tLULegCOM(0,3) + tRULegCOM(0,3))+
         mLowerLeg * (tLLLegCOM(0,3) + tRLLegCOM(0,3))+
         mFoot *  (tLFootCOM(0,3)+ tRFootCOM(0,3));



  r[1] = mPelvis * tPelvisCOM(1,3) +
         mTorso * tTorsoCOM(1,3) +
         mUpperArm * (tLUArmCOM(1,3) + tRUArmCOM(1,3))+
         mElbow * (tLElbowCOM(1,3) + tRElbowCOM(1,3))+
         mLowerArm * (tLLArmCOM(1,3)+ tRLArmCOM(1,3))+
         mWrist * (tLWristCOM(1,3) + tRWristCOM(1,3))+
         mLHand * tLHandCOM(1,3) + 
         mRHand * tRHandCOM(1,3) +

         mUpperLeg * (tLULegCOM(1,3) + tRULegCOM(1,3))+
         mLowerLeg * (tLLLegCOM(1,3) + tRLLegCOM(1,3))+
         mFoot *  (tLFootCOM(1,3)+ tRFootCOM(1,3));

  r[2] = mPelvis * tPelvisCOM(2,3) +
         mTorso * tTorsoCOM(2,3) +
         mUpperArm * (tLUArmCOM(2,3) + tRUArmCOM(2,3))+
         mElbow * (tLElbowCOM(2,3) + tRElbowCOM(2,3))+
         mLowerArm * (tLLArmCOM(2,3)+ tRLArmCOM(2,3))+
         mWrist * (tLWristCOM(2,3) + tRWristCOM(2,3))+
         mLHand * tLHandCOM(2,3) + 
         mRHand * tRHandCOM(2,3) +

         mUpperLeg * (tLULegCOM(2,3) + tRULegCOM(2,3))+
         mLowerLeg * (tLLLegCOM(2,3) + tRLLegCOM(2,3))+
         mFoot *  (tLFootCOM(2,3)+ tRFootCOM(2,3));


  r[3] = mPelvis + mTorso + 
      2* (
        mUpperArm + mElbow + mLowerArm + mWrist +
        mUpperLeg +mLowerLeg + mFoot
        ) + 
      mLHand + mRHand;

//TODO: we should ignore support foot mass 


  return r;




/*
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
*/  
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