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



void THOROP_kinematics_calculate_arm_torque(
  double* stall_torque, double* acc_torque,
  const double *rpyangle,const double *qArm, const double *qArmAcc){


  Transform 
    COM0,COM1,COM2,COM3,COM4,COM5,COM6,

    Jac00,
    Jac10,Jac11,
    Jac20,Jac21,Jac22,
    Jac30,Jac31,Jac32,Jac33,
    Jac40,Jac41,Jac42,Jac43,Jac44,
    Jac50,Jac51,Jac52,Jac53,Jac54,Jac55,
    Jac60,Jac61,Jac62,Jac63,Jac64,Jac65,Jac66;
  
  for (int i=0;i<7;i++) {stall_torque[i]=0;acc_torque[i]=0;}

  Transform torso;
  torso.rotateX(rpyangle[0]).rotateY(rpyangle[1]);

  //more compact calculation
  COM0 = trcopy(torso).rotateY(qArm[0]).translate(armCom[0]);
  Jac00= trcopy(torso).rotateDotY(qArm[0]);  //d (com0) / dq0

  COM1= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armCom[1]);
  Jac10= trcopy(torso).rotateDotY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armCom[1]);
  Jac11= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateDotZ(qArm[1]).translate(armCom[1]);

  COM2= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armCom[2]);
  Jac20= trcopy(torso).rotateDotY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armCom[2]);
  Jac21= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateDotZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armCom[2]);
  Jac22= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateDotX(qArm[2]).translate(armCom[2]);

  COM3= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armCom[3]);
  Jac30= trcopy(torso).rotateDotY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armCom[3]);
  Jac31= trcopy(torso).rotateY(qArm[0]).translate(armLink[1]) 
        .rotateDotZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armCom[3]);
  Jac32= trcopy(torso).rotateY(qArm[0]).translate(armLink[1]) 
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateDotX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armCom[3]);
  Jac33= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateDotY(qArm[3]).translate(armCom[3]);

  COM4= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armCom[4]);
  Jac40= trcopy(torso).rotateDotY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armCom[4]);
  Jac41= trcopy(torso).rotateY(qArm[0]).translate(armLink[1]) 
        .rotateDotZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armCom[4]);
  Jac42= trcopy(torso).rotateY(qArm[0]).translate(armLink[1]) 
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateDotX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armCom[4]);
  Jac43= trcopy(torso).rotateY(qArm[0]).translate(armLink[1]) 
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateDotY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armCom[4]);
  Jac44= trcopy(torso).rotateY(qArm[0]).translate(armLink[1]) 
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateDotX(qArm[4]).translate(armCom[4]);

  COM5= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armCom[5]);
  Jac50= trcopy(torso).rotateDotY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armCom[5]);
  Jac51= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateDotZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armCom[5]);
  Jac52= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateDotX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armCom[5]);
  Jac53= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateDotY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armCom[5]);
  Jac54= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateDotX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armCom[5]);
  Jac55= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateDotZ(qArm[5]).translate(armCom[5]);

  COM6= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(armCom[6]);
  Jac60= trcopy(torso).rotateDotY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(armCom[6]);
  Jac61= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateDotZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(armCom[6]);
  Jac62= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateDotX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(armCom[6]);
  Jac63= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateDotY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(armCom[6]);
  Jac64= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateDotX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(armCom[6]);
  Jac65= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateDotZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(armCom[6]);
  Jac66= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateDotX(qArm[6]).translate(armCom[6]);

  Transform JacZZ;
  Jacobian J0,J1,J2,J3,J4,J5,J6;
  J0.calculate7(COM0,Jac00,JacZZ,JacZZ,JacZZ,JacZZ,JacZZ,JacZZ);  
  J1.calculate7(COM1,Jac10,Jac11,JacZZ,JacZZ,JacZZ,JacZZ,JacZZ);
  J2.calculate7(COM2,Jac20,Jac21,Jac22,JacZZ,JacZZ,JacZZ,JacZZ);  
  J3.calculate7(COM3,Jac30,Jac31,Jac32,Jac33,JacZZ,JacZZ,JacZZ);
  J4.calculate7(COM4,Jac40,Jac41,Jac42,Jac43,Jac44,JacZZ,JacZZ);  
  J5.calculate7(COM5,Jac50,Jac51,Jac52,Jac53,Jac54,Jac55,JacZZ);
  J6.calculate7(COM6,Jac60,Jac61,Jac62,Jac63,Jac64,Jac65,Jac66);
  
  J0.accumulate_stall_torque(stall_torque, 0.0,0.0,MassArm[0]*g);
  J1.accumulate_stall_torque(stall_torque, 0.0,0.0,MassArm[1]*g);
  J2.accumulate_stall_torque(stall_torque, 0.0,0.0,MassArm[2]*g);
  J3.accumulate_stall_torque(stall_torque, 0.0,0.0,MassArm[3]*g);
  J4.accumulate_stall_torque(stall_torque, 0.0,0.0,MassArm[4]*g);
  J5.accumulate_stall_torque(stall_torque, 0.0,0.0,MassArm[5]*g);
  J6.accumulate_stall_torque(stall_torque, 0.0,0.0,MassArm[6]*g);

  J0.accumulate_acc_torque(acc_torque, &qArmAcc[0], MassArm[0], &InertiaArm[0][0]);
  J1.accumulate_acc_torque(acc_torque, &qArmAcc[0], MassArm[1], &InertiaArm[1][0]);
  J2.accumulate_acc_torque(acc_torque, &qArmAcc[0], MassArm[2], &InertiaArm[2][0]);
  J3.accumulate_acc_torque(acc_torque, &qArmAcc[0], MassArm[3], &InertiaArm[3][0]);
  J4.accumulate_acc_torque(acc_torque, &qArmAcc[0], MassArm[4], &InertiaArm[4][0]);
  J5.accumulate_acc_torque(acc_torque, &qArmAcc[0], MassArm[5], &InertiaArm[5][0]);
  J6.accumulate_acc_torque(acc_torque, &qArmAcc[0], MassArm[6], &InertiaArm[6][0]);
  
}



void THOROP_kinematics_calculate_leg_torque(
  double* stall_torque, double* acc_torque,
  const double *rpyangle,const double *qLeg,const double *qLegAcc,
  int isLeft, double grf, const double *support){

  int index = 6;
  if (isLeft>0) index = 0;

  Transform 
    COM0,COM1,COM2,COM3,COM4,COM5,COMS,
    Jac00,
    Jac10,Jac11,
    Jac20,Jac21,Jac22,
    Jac30,Jac31,Jac32,Jac33,
    Jac40,Jac41,Jac42,Jac43,Jac44,
    Jac50,Jac51,Jac52,Jac53,Jac54,Jac55,
    JacS0,JacS1,JacS2,JacS3,JacS4,JacS5;

  for (int i=0;i<6;i++) {stall_torque[i]=0;acc_torque[i]=0;
    
  Transform torso;
  torso.rotateX(rpyangle[0]).rotateY(rpyangle[1]);

  COM0= trcopy(torso).rotateZ(qLeg[0]).translate(legCom[index]);
  Jac00=trcopy(torso).rotateDotZ(qLeg[0]).translate(legCom[index]);

  COM1= trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legCom[index+1]);
  Jac10=trcopy(torso).rotateDotZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legCom[index+1]);
  Jac11=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateDotX(qLeg[1]).translate(legCom[index+1]);       

  COM2= trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legCom[index+2]);
  Jac20=trcopy(torso).rotateDotZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legCom[index+2]);
  Jac21=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateDotX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legCom[index+2]);
  Jac22=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateDotY(qLeg[2]).translate(legCom[index+2]);

  COM3= trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legCom[index+3]);
  Jac30=trcopy(torso).rotateDotZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legCom[index+3]);
  Jac31=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateDotX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legCom[index+3]);
  Jac32=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateDotY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legCom[index+3]);
  Jac33=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateDotY(qLeg[3]).translate(legCom[index+3]);

  COM4=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legCom[index+4]);
  Jac40=trcopy(torso).rotateDotZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legCom[index+4]);
  Jac41=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateDotX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legCom[index+4]);
  Jac42=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateDotY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legCom[index+4]);
  Jac43=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateDotY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legCom[index+4]);
  Jac44=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateDotY(qLeg[4]).translate(legCom[index+4]);

  COM5=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legLink[5])
       .rotateX(qLeg[5]).translate(legCom[index+5]);
  Jac50=trcopy(torso).rotateDotZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legLink[5])
       .rotateX(qLeg[5]);  
  Jac51=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateDotX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legLink[5])
       .rotateX(qLeg[5]);
  Jac52=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateDotY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legLink[5])
       .rotateX(qLeg[5]);
  Jac53=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateDotY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legLink[5])
       .rotateX(qLeg[5]);
  Jac54=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateDotY(qLeg[4]).translate(legLink[5])
       .rotateX(qLeg[5]);
  Jac55=trcopy(torso).rotateZ(qLeg[0]).translate(legLink[1])
       .rotateX(qLeg[1]).translate(legLink[2])
       .rotateY(qLeg[2]).translate(legLink[3])
       .rotateY(qLeg[3]).translate(legLink[4])
       .rotateY(qLeg[4]).translate(legLink[5])
       .rotateDotX(qLeg[5]);

  COMS = trcopy(COM5).translate(support[0],support[1],support[2]).translate(legLink[6]);
  JacS0 = trcopy(Jac50).translate(support[0],support[1],support[2]).translate(legLink[6]);
  JacS1 = trcopy(Jac51).translate(support[0],support[1],support[2]).translate(legLink[6]);
  JacS2 = trcopy(Jac52).translate(support[0],support[1],support[2]).translate(legLink[6]);
  JacS3 = trcopy(Jac53).translate(support[0],support[1],support[2]).translate(legLink[6]);
  JacS4 = trcopy(Jac54).translate(support[0],support[1],support[2]).translate(legLink[6]);
  JacS5 = trcopy(Jac55).translate(support[0],support[1],support[2]).translate(legLink[6]);

  Jac50.translate(legCom[index+5]);
  Jac51.translate(legCom[index+5]);
  Jac52.translate(legCom[index+5]);
  Jac53.translate(legCom[index+5]);
  Jac54.translate(legCom[index+5]);
  Jac55.translate(legCom[index+5]);

  Transform JacZZ;
  Jacobian J0,J1,J2,J3,J4,J5,JS;
  
  J0.calculate6(COM0,Jac00,JacZZ,JacZZ,JacZZ,JacZZ,JacZZ);  
  J1.calculate6(COM1,Jac10,Jac11,JacZZ,JacZZ,JacZZ,JacZZ);
  J2.calculate6(COM2,Jac20,Jac21,Jac22,JacZZ,JacZZ,JacZZ);  
  J3.calculate6(COM3,Jac30,Jac31,Jac32,Jac33,JacZZ,JacZZ);
  J4.calculate6(COM4,Jac40,Jac41,Jac42,Jac43,Jac44,JacZZ);  
  J5.calculate6(COM5,Jac50,Jac51,Jac52,Jac53,Jac54,Jac55);
  JS.calculate6(COMS,JacS0,JacS1,JacS2,JacS3,JacS4,JacS5);

  J0.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[0]*g);
  J1.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[1]*g);
  J2.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[2]*g);
  J3.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[3]*g);
  J4.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[4]*g);
  J5.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[5]*g);
  JS.accumulate_stall_torque(stall_torque, 0.0,0.0,-grf);
  

  J0.accumulate_acc_torque(acc_torque, &qLegAcc[0], MassLeg[0], &InertiaLeg[index+0][0]);
  J1.accumulate_acc_torque(acc_torque, &qLegAcc[0], MassLeg[1], &InertiaLeg[index+1][0]);
  J2.accumulate_acc_torque(acc_torque, &qLegAcc[0], MassLeg[2], &InertiaLeg[index+2][0]);
  J3.accumulate_acc_torque(acc_torque, &qLegAcc[0], MassLeg[3], &InertiaLeg[index+3][0]);
  J4.accumulate_acc_torque(acc_torque, &qLegAcc[0], MassLeg[4], &InertiaLeg[index+4][0]);
  J5.accumulate_acc_torque(acc_torque, &qLegAcc[0], MassLeg[5], &InertiaLeg[index+5][0]);
  }
}