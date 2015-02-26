#include "THOROPKinematics.h"



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







//Arm collision check
int THOROP_kinematics_check_collision(const double *qLArm,const double *qRArm){


  /* inverse kinematics to convert joint angles to servo positions */
  std::vector<double> r(4);

  Transform tTorso;

  Transform tLShoulder = trcopy(tTorso)
    .translateY(shoulderOffsetY)
    .translateZ(shoulderOffsetZ)        
    .rotateY(qLArm[0])
    .rotateZ(qLArm[1])
    .rotateX(qLArm[2]);

  Transform tLElbow = trcopy(tLShoulder)
    .translateX(upperArmLength)
    .translateZ(elbowOffsetX)
    .rotateY(qLArm[3]);
  
  Transform tLWrist = trcopy(tLElbow)
    .translateZ(-elbowOffsetX)
    .rotateX(qLArm[4]);         
  
  Transform tLHand = trcopy(tLWrist)
    .translateX(lowerArmLength)          
    .rotateZ(qLArm[5])
    .rotateX(qLArm[6]);

  Transform tRShoulder = trcopy(tTorso)
    .translateY(-shoulderOffsetY)
    .translateZ(shoulderOffsetZ)        
    .rotateY(qRArm[0])
    .rotateZ(qRArm[1])
    .rotateX(qRArm[2]);

  Transform tRElbow = trcopy(tRShoulder)
    .translateX(upperArmLength)
    .translateZ(elbowOffsetX)
    .rotateY(qRArm[3]);    

  Transform tRWrist = trcopy(tRElbow)
    .translateZ(-elbowOffsetX)
    .rotateX(qRArm[4]);            

  Transform tRHand = trcopy(tRWrist)
    .translateX(lowerArmLength)          
    .rotateZ(qRArm[5])
    .rotateX(qRArm[6]);

  //Elbow should always be outside the shoulder
  if ((tLElbow(1,3)<tLShoulder(1,3))||(tRElbow(1,3)>tRShoulder(1,3))){
    printf("Elbow collision!\n");
    return 1;
  }
  return 0;
}


//Arm collision check
int THOROP_kinematics_check_collision_single(const double *qArm,int is_left){


  /* inverse kinematics to convert joint angles to servo positions */
  std::vector<double> r(4);

  Transform tTorso;

  Transform tShoulder;

  if (is_left>0)
    tShoulder = trcopy(tTorso)
      .translateY(shoulderOffsetY)
      .translateZ(shoulderOffsetZ)        
      .rotateY(qArm[0])
      .rotateZ(qArm[1])
      .rotateX(qArm[2]);
  else
    tShoulder = trcopy(tTorso)
      .translateY(-shoulderOffsetY)
      .translateZ(shoulderOffsetZ)        
      .rotateY(qArm[0])
      .rotateZ(qArm[1])
      .rotateX(qArm[2]);


  Transform tElbow = trcopy(tShoulder)
    .translateX(upperArmLength)
    .translateZ(elbowOffsetX)
    .rotateY(qArm[3]);
  
  Transform tWrist = trcopy(tElbow)
    .translateZ(-elbowOffsetX)
    .rotateX(qArm[4]);         

/*  
  Transform tLHand = trcopy(tLWrist)
    .translateX(lowerArmLength)          
    .rotateZ(qLArm[5])
    .rotateX(qLArm[6]);
*/

  //Elbow should always be outside the shoulder
  if (fabs(tElbow(1,3)) <fabs( tShoulder(1,3))  ){
    //printf("Elbow collision!\n");
    return 1;
  }
  //Wrist collision check with pelvis

  //Pelvis box: 





  return 0;
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




std::vector<double>
THOROP_kinematics_calculate_arm_torque(const double *qArm){

  Transform tShoulder;

//COM jacobian matrix

  Transform 
    Jac00,
    Jac10,Jac11,
    Jac20,Jac21,Jac22,
    Jac30,Jac31,Jac32,Jac33,
    Jac40,Jac41,Jac42,Jac43,Jac44,
    Jac50,Jac51,Jac52,Jac53,Jac54,Jac55,
    Jac60,Jac61,Jac62,Jac63,Jac64,Jac65,Jac66;

  //more compact calculation
  Jac00.rotateDotY(qArm[0]);  //d (com0) / dq0

  Jac10.rotateDotY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armCom[1]);
  Jac11.rotateY(qArm[0]).translate(armLink[1])
        .rotateDotZ(qArm[1]).translate(armCom[1]);

  Jac20.rotateDotY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armCom[2]);
  Jac21.rotateY(qArm[0]).translate(armLink[1])
        .rotateDotZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armCom[2]);
  Jac22.rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateDotX(qArm[2]).translate(armCom[2]);

  Jac30.rotateDotY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armCom[3]);
  Jac31.rotateY(qArm[0]).translate(armLink[1]) 
        .rotateDotZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armCom[3]);
  Jac32.rotateY(qArm[0]).translate(armLink[1]) 
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateDotX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armCom[3]);
  Jac33.rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateDotY(qArm[3]).translate(armCom[3]);

  Jac40.rotateDotY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armCom[4]);
  Jac41.rotateY(qArm[0]).translate(armLink[1]) 
        .rotateDotZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armCom[4]);
  Jac42.rotateY(qArm[0]).translate(armLink[1]) 
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateDotX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armCom[4]);
  Jac43.rotateY(qArm[0]).translate(armLink[1]) 
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateDotY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armCom[4]);
  Jac44.rotateY(qArm[0]).translate(armLink[1]) 
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateDotX(qArm[4]).translate(armCom[4]);

  Jac50.rotateDotY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armCom[5]);
  Jac51.rotateY(qArm[0]).translate(armLink[1])
        .rotateDotZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armCom[5]);
  Jac52.rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateDotX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armCom[5]);
  Jac53.rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateDotY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armCom[5]);
  Jac54.rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateDotX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armCom[5]);
  Jac55.rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateDotZ(qArm[5]).translate(armCom[5]);

  Jac60.rotateDotY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(armCom[6]);
  Jac61.rotateY(qArm[0]).translate(armLink[1])
        .rotateDotZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(armCom[6]);
  Jac62.rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateDotX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(armCom[6]);
  Jac63.rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateDotY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(armCom[6]);
  Jac64.rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateDotX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(armCom[6]);
  Jac65.rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateDotZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(armCom[6]);
  Jac66.rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateDotX(qArm[6]).translate(armCom[6]);


//2.54 0.17 -0.04 -0.12 0.00  

  std::vector<double> torque(7);

  
  torque[0] = 
    Jac00.getZ() * MassArm[0]+
    Jac10.getZ() * MassArm[1]+
    Jac20.getZ() * MassArm[2]+
    Jac30.getZ() * MassArm[3]+
    Jac40.getZ() * MassArm[4]+
    Jac50.getZ() * MassArm[5]+
    Jac60.getZ() * MassArm[6];
    

  torque[1] =     
    Jac11.getZ() * MassArm[1]+
    Jac21.getZ() * MassArm[2]+
    Jac31.getZ() * MassArm[3]+
    Jac41.getZ() * MassArm[4]+
    Jac51.getZ() * MassArm[5]+
    Jac61.getZ() * MassArm[6];
        

  torque[2] =         
    Jac22.getZ() * MassArm[2]+
    Jac32.getZ() * MassArm[3]+
    Jac42.getZ() * MassArm[4]+
    Jac52.getZ() * MassArm[5]+
    Jac62.getZ() * MassArm[6];
    

  torque[3] =             
    Jac33.getZ() * MassArm[3]+
    Jac43.getZ() * MassArm[4]+
    Jac53.getZ() * MassArm[5]+
    Jac63.getZ() * MassArm[6];
    

  torque[4] =             
    Jac44.getZ() * MassArm[4]+
    Jac54.getZ() * MassArm[5]+
    Jac64.getZ() * MassArm[6];
    
  torque[5] =                 
    Jac55.getZ() * MassArm[5]+
    Jac65.getZ() * MassArm[6];
  
  torque[6] =                 
    Jac66.getZ() * MassArm[6];
  

  //Torque = (g*m)' J_i  
  for (int i=0;i<7;i++) torque[i]*=g;

  return torque;
}



std::vector<double> THOROP_kinematics_calculate_leg_torque(const double *qLeg,int isLeft, double grf, const double *support){
  int index = 6;
  if (isLeft>0) index = 0;


  Transform 
    Jac00,
    Jac10,Jac11,
    Jac20,Jac21,Jac22,
    Jac30,Jac31,Jac32,Jac33,
    Jac40,Jac41,Jac42,Jac43,Jac44,
    Jac50,Jac51,Jac52,Jac53,Jac54,Jac55,
    JacS0,JacS1,JacS2,JacS3,JacS4,JacS5;
    

  


  Jac00.rotateDotZ(qLeg[0]).translate(legCom[index]);

  Jac10.rotateDotZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legCom[index+1]);
  Jac11.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateDotX(qLeg[1]).translate(legCom[index+1]);       

  Jac20.rotateDotZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legCom[index+2]);
  Jac21.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateDotX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legCom[index+2]);
  Jac22.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateDotY(qLeg[2]).translate(legCom[index+2]);

  Jac30.rotateDotZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legCom[index+3]);
  Jac31.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateDotX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legCom[index+3]);
  Jac32.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateDotY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legCom[index+3]);
  Jac33.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateDotY(qLeg[3]).translate(legCom[index+3]);

  Jac40.rotateDotZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legCom[index+4]);
  Jac41.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateDotX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legCom[index+4]);
  Jac42.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateDotY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legCom[index+4]);
  Jac43.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateDotY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legCom[index+4]);
  Jac44.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateDotY(qLeg[4]).translate(legCom[index+4]);

  Jac50.rotateDotZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legLink[5]);


  
  Jac51.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateDotX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legLink[5])
       .rotateX(qLeg[5]);
  Jac52.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateDotY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legLink[5])
       .rotateX(qLeg[5]);
  Jac53.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateDotY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legLink[5])
       .rotateX(qLeg[5]);
  Jac54.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateDotY(qLeg[4]).translate(legLink[5])
       .rotateX(qLeg[5]);
  Jac55.rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legLink[5])
       .rotateDotX(qLeg[5]);


  JacS0 = trcopy(Jac50).translate(support[0],support[1],support[2]);
  JacS1 = trcopy(Jac51).translate(support[0],support[1],support[2]);
  JacS2 = trcopy(Jac52).translate(support[0],support[1],support[2]);
  JacS3 = trcopy(Jac53).translate(support[0],support[1],support[2]);
  JacS4 = trcopy(Jac54).translate(support[0],support[1],support[2]);
  JacS5 = trcopy(Jac55).translate(support[0],support[1],support[2]);

  Jac50.translate(legCom[index+5]);
  Jac51.translate(legCom[index+5]);
  Jac52.translate(legCom[index+5]);
  Jac53.translate(legCom[index+5]);
  Jac54.translate(legCom[index+5]);
  Jac55.translate(legCom[index+5]);

  std::vector<double> torque(6);

  
//  double net_grf = grf/g-MassLeg[0]-MassLeg[1]-MassLeg[2]-MassLeg[3]-MassLeg[4]-MassLeg[5];
  double net_grf= grf/g;

//vertical grf act as negative gravitational force
/*
  torque[0] = 
    Jac00.getZ() * MassLeg[0]+
    Jac10.getZ() * MassLeg[1]+
    Jac20.getZ() * MassLeg[2]+
    Jac30.getZ() * MassLeg[3]+
    Jac40.getZ() * MassLeg[4]+
    Jac50.getZ() * MassLeg[5]
    -JacS0.getZ()* net_grf;

  torque[1] =     
    Jac11.getZ() * MassLeg[1]+
    Jac21.getZ() * MassLeg[2]+
    Jac31.getZ() * MassLeg[3]+
    Jac41.getZ() * MassLeg[4]+
    Jac51.getZ() * MassLeg[5]
    -JacS1.getZ()* net_grf;
        
  torque[2] =         
    Jac22.getZ() * MassLeg[2]+
    Jac32.getZ() * MassLeg[3]+
    Jac42.getZ() * MassLeg[4]+
    Jac52.getZ() * MassLeg[5]
    -JacS2.getZ()* net_grf;

  torque[3] =             
    Jac33.getZ() * MassLeg[3]+
    Jac43.getZ() * MassLeg[4]+
    Jac53.getZ() * MassLeg[5]
    -JacS3.getZ()* net_grf;
    

  torque[4] =             
    Jac44.getZ() * MassLeg[4]+
    Jac54.getZ() * MassLeg[5]
    -JacS4.getZ()* net_grf;
    
  torque[5] =                 
    Jac55.getZ() * MassLeg[5]
    -JacS5.getZ()* net_grf;
 */

torque[0] = 
    Jac00.getZ() * MassLeg[0]+
    Jac10.getZ() * MassLeg[1]+
    Jac20.getZ() * MassLeg[2]+
    Jac30.getZ() * MassLeg[3]+
    Jac40.getZ() * MassLeg[4]+
    Jac50.getZ() * MassLeg[5]
    -JacS0.getZ()* net_grf;

  torque[1] =     
    Jac11.getZ() * MassLeg[1]+
    Jac21.getZ() * MassLeg[2]+
    Jac31.getZ() * MassLeg[3]+
    Jac41.getZ() * MassLeg[4]+
    Jac51.getZ() * MassLeg[5]
    -JacS1.getZ()* net_grf;
        
  torque[2] =         
    Jac22.getZ() * MassLeg[2]+
    Jac32.getZ() * MassLeg[3]+
    Jac42.getZ() * MassLeg[4]+
    Jac52.getZ() * MassLeg[5]
    -JacS2.getZ()* net_grf;

  torque[3] =             
    Jac33.getZ() * MassLeg[3]+
    Jac43.getZ() * MassLeg[4]+
    Jac53.getZ() * MassLeg[5]
    -JacS3.getZ()* net_grf;
    

  torque[4] =             
    Jac44.getZ() * MassLeg[4]+
    Jac54.getZ() * MassLeg[5]
    -JacS4.getZ()* (net_grf);
    
  torque[5] =                 
    Jac55.getZ() * MassLeg[5]
    -JacS5.getZ()* (net_grf);

  //Torque = (g*m)' J_i  
  for (int i=0;i<6;i++) torque[i]*=g;
  return torque;
}
