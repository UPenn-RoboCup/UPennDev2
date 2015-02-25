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


  tLLeg0 = trcopy(tPelvis).translate(legLink[0])
          .rotateZ(qLLeg[0])
          .translate(legCom[0]);
  tLLeg1 = trcopy(tPelvis).translate(legLink[0])
          .rotateZ(qLLeg[0]).rotateX(qLLeg[1])
          .translate(legCom[1]);
  tLLeg2 = trcopy(tPelvis).translate(legLink[0])
          .rotateZ(qLLeg[0]).rotateX(qLLeg[1]).rotateY(qLLeg[2])
          .translate(legCom[2]);
  tLLeg3 = trcopy(tPelvis).translate(legLink[0])
          .rotateZ(qLLeg[0]).rotateX(qLLeg[1]).rotateY(qLLeg[2])
          .translate(legLink[3]).rotateY(qLLeg[3])
          .translate(legCom[3]);
  tLLeg4 = trcopy(tPelvis).translate(legLink[0])
          .rotateZ(qLLeg[0]).rotateX(qLLeg[1]).rotateY(qLLeg[2])
          .translate(legLink[3]).rotateY(qLLeg[3])
          .translate(legLink[4]).rotateY(qLLeg[4])
          .translate(legCom[4]);
  tLLeg5 = trcopy(tPelvis).translate(legLink[0])
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


std::vector<double> THOROP_kinematics_calculate_leg_torque(const double *qLeg,int isLeft, const double *com_rest){
  int index = 0;
  const double *legLink0=rlegLink0;
  if (isLeft>0) {
    index = 6;
    legLink0=llegLink0;
  }

  double bodyCom[3]={
    com_rest[0]/com_rest[3],com_rest[1]/com_rest[3],com_rest[2]/com_rest[3]
  };


  Transform 
    Jac45,
    Jac34,Jac35,
    Jac23,Jac24,Jac25,
    Jac12,Jac13,Jac14,Jac15,
    Jac01,Jac02,Jac03,Jac04,Jac05,
    JacB0,JacB1,JacB2,JacB3,JacB4,JacB5;

    //Com 4
    Jac45.invtranslate(legLink[6]).rotateDotX(-qLeg[5]).neg()
     .invtranslate(legLink[5]).translate(legCom[4+index]); 

    //COM 3
    Jac34.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateDotY(-qLeg[4]).neg()
         .invtranslate(legLink[4]).translate(legCom[3+index]);

    Jac35.invtranslate(legLink[6]).rotateDotX(-qLeg[5]).neg()
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).translate(legCom[3+index]);

    //COM 2
    Jac23.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).rotateDotY(-qLeg[3]).neg()
         .invtranslate(legLink[3]).translate(legCom[2+index]);
    Jac24.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateDotY(-qLeg[4]).neg()
         .invtranslate(legLink[4]).rotateY(-qLeg[3])
         .invtranslate(legLink[3]).translate(legCom[2+index]);
    Jac25.invtranslate(legLink[6]).rotateDotX(-qLeg[5]).neg()
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).rotateY(-qLeg[3])
         .invtranslate(legLink[3]).translate(legCom[2+index]);

    //COM 1
    Jac12.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).rotateY(-qLeg[3])
         .invtranslate(legLink[3]).rotateDotY(-qLeg[2]).neg()
         .invtranslate(legLink[2]).translate(legCom[1+index]);
    Jac13.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).rotateDotY(-qLeg[3]).neg()
         .invtranslate(legLink[3]).rotateY(-qLeg[2])
         .invtranslate(legLink[2]).translate(legCom[1+index]);
    Jac14.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateDotY(-qLeg[4]).neg()
         .invtranslate(legLink[4]).rotateY(-qLeg[3])
         .invtranslate(legLink[3]).rotateY(-qLeg[2])
         .invtranslate(legLink[2]).translate(legCom[1+index]);
    Jac15.invtranslate(legLink[6]).rotateDotX(-qLeg[5]).neg()
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).rotateY(-qLeg[3])
         .invtranslate(legLink[3]).rotateY(-qLeg[2])
         .invtranslate(legLink[2]).translate(legCom[1+index]);

    //COM 0
    Jac01.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).rotateY(-qLeg[3])
         .invtranslate(legLink[3]).rotateY(-qLeg[2])
         .invtranslate(legLink[2]).rotateDotX(-qLeg[1]).neg()
         .invtranslate(legLink[1]).translate(legCom[0+index]);
    Jac02.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).rotateY(-qLeg[3])
         .invtranslate(legLink[3]).rotateDotY(-qLeg[2]).neg()
         .invtranslate(legLink[2]).rotateX(-qLeg[1])
         .invtranslate(legLink[1]).translate(legCom[0+index]);
    Jac03.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).rotateDotY(-qLeg[3]).neg()
         .invtranslate(legLink[3]).rotateY(-qLeg[2])
         .invtranslate(legLink[2]).rotateX(-qLeg[1])
         .invtranslate(legLink[1]).translate(legCom[0+index]);         
    Jac04.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateDotY(-qLeg[4]).neg()
         .invtranslate(legLink[4]).rotateY(-qLeg[3])
         .invtranslate(legLink[3]).rotateY(-qLeg[2])
         .invtranslate(legLink[2]).rotateX(-qLeg[1])
         .invtranslate(legLink[1]).translate(legCom[0+index]);
    Jac05.invtranslate(legLink[6]).rotateDotX(-qLeg[5]).neg()
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).rotateY(-qLeg[3])
         .invtranslate(legLink[3]).rotateY(-qLeg[2])
         .invtranslate(legLink[2]).rotateX(-qLeg[1])
         .invtranslate(legLink[1]).translate(legCom[0+index]);         

    //Body
    JacB0.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).rotateY(-qLeg[3])
         .invtranslate(legLink[3]).rotateY(-qLeg[2])
         .invtranslate(legLink[2]).rotateX(-qLeg[1])
         .invtranslate(legLink[1]).rotateDotZ(-qLeg[0]).neg()
         .invtranslate(legLink0).translate(bodyCom);
    JacB1.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).rotateY(-qLeg[3])
         .invtranslate(legLink[3]).rotateY(-qLeg[2])
         .invtranslate(legLink[2]).rotateDotX(-qLeg[1]).neg()
         .invtranslate(legLink[1]).rotateZ(-qLeg[0])
         .invtranslate(legLink0).translate(bodyCom);
    JacB2.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).rotateY(-qLeg[3])
         .invtranslate(legLink[3]).rotateDotY(-qLeg[2]).neg()
         .invtranslate(legLink[2]).rotateX(-qLeg[1])
         .invtranslate(legLink[1]).rotateZ(-qLeg[0])
         .invtranslate(legLink0).translate(bodyCom);
    JacB3.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).rotateDotY(-qLeg[3]).neg()
         .invtranslate(legLink[3]).rotateY(-qLeg[2])
         .invtranslate(legLink[2]).rotateX(-qLeg[1])
         .invtranslate(legLink[1]).rotateZ(-qLeg[0])
         .invtranslate(legLink0).translate(bodyCom);
    JacB4.invtranslate(legLink[6]).rotateX(-qLeg[5])
         .invtranslate(legLink[5]).rotateDotY(-qLeg[4]).neg()
         .invtranslate(legLink[4]).rotateY(-qLeg[3])
         .invtranslate(legLink[3]).rotateY(-qLeg[2])
         .invtranslate(legLink[2]).rotateX(-qLeg[1])
         .invtranslate(legLink[1]).rotateZ(-qLeg[0])
         .invtranslate(legLink0).translate(bodyCom);
    JacB5.invtranslate(legLink[6]).rotateDotX(-qLeg[5]).neg()
         .invtranslate(legLink[5]).rotateY(-qLeg[4])
         .invtranslate(legLink[4]).rotateY(-qLeg[3])
         .invtranslate(legLink[3]).rotateY(-qLeg[2])
         .invtranslate(legLink[2]).rotateX(-qLeg[1])
         .invtranslate(legLink[1]).rotateZ(-qLeg[0])
         .invtranslate(legLink0).translate(bodyCom);



  std::vector<double> torque(6);   
  
  double MassBody = com_rest[3];

  torque[0] =     
    JacB0.getZ() * MassBody;

  torque[1] =     
    JacB1.getZ() * MassBody+
    Jac01.getZ() * MassLeg[0];
    
  torque[2] =     
    JacB2.getZ() * MassBody+
    Jac02.getZ() * MassLeg[0]+
    Jac12.getZ() * MassLeg[1];

  torque[3] =     
    JacB3.getZ() * MassBody+
    Jac03.getZ() * MassLeg[0]+
    Jac13.getZ() * MassLeg[1]+
    Jac23.getZ() * MassLeg[2];

  torque[4] =     
    JacB4.getZ() * MassBody+
    Jac04.getZ() * MassLeg[0]+
    Jac14.getZ() * MassLeg[1]+
    Jac24.getZ() * MassLeg[2]+
    Jac34.getZ() * MassLeg[3];

  torque[5] =     
    JacB5.getZ() * MassBody+
    Jac05.getZ() * MassLeg[0]+
    Jac15.getZ() * MassLeg[1]+
    Jac25.getZ() * MassLeg[2]+
    Jac35.getZ() * MassLeg[3]+
    Jac45.getZ() * MassLeg[4];

  //Torque = (g*m)' J_i  
  for (int i=0;i<7;i++) torque[i]*=g;
  return torque;
}
