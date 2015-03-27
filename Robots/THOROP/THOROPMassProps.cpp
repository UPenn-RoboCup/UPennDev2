#include "THOROPKinematics.h"

void THOROP_kinematics_calculate_arm_com(const double* rpyangle,  
   const double *qArm, int index,double *comxyz, double*comrpy){  

  Transform torso, COM;
  torso.rotateX(rpyangle[0]).rotateY(rpyangle[1]);

  switch (index){
  case 0:
    COM = trcopy(torso).rotateY(qArm[0]).translate(armCom[0]);
    break;
  case 1:
    COM= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
         .rotateZ(qArm[1]).translate(armCom[1]);
    break;
  case 2:
    COM= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armCom[2]);
    break;
  case 3:
    COM= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armCom[3]);      
    break;        
  case 4:
    COM= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armCom[4]);
    break;        
  case 5:
    COM= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armCom[5]);
    break;        
  case 6:
    COM= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(armCom[6]);
    break;
  }  
  COM.getXYZ(&comxyz[0]);
  Transform COMinv = inv(COM);
  getAngularVelocityTensor(COM,COMinv,&comrpy[0]);
}



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


void THOROP_kinematics_calculate_arm_torque_adv(
  double* stall_torque,double* acc_torque,double* acc_torque2,const double *rpyangle,
  const double *qArm,const double *qArmVel,const double *qArmAcc,double dq){

  double dummy[7];
  double qArmDq[7];
  double b_matrix0[49];
  double b_matrix[7][49];

  int i,j,k;
  for (i=0;i<7;i++) {
    qArmDq[i]=qArm[i];
    stall_torque[i]=0;
    acc_torque[i]=0;
    acc_torque2[i]=0;
  }

  THOROP_kinematics_calculate_arm_torque(stall_torque, b_matrix0, rpyangle,qArm);    

/*
  printf("B matrix:\n");
  for(i=0;i<7;i++){ 
    for (j=0;j<7;j++)
      printf("%.6f ",b_matrix0[i*7+j]);
    printf("\n");
  }
  printf("\n");
*/

  qArmDq[0]=qArm[0]+dq;  
  THOROP_kinematics_calculate_arm_torque(dummy, b_matrix[0], rpyangle,qArmDq);
  qArmDq[0]=qArm[0];  
  qArmDq[1]=qArm[1]+dq;  
  THOROP_kinematics_calculate_arm_torque(dummy, b_matrix[1], rpyangle,qArmDq);
  qArmDq[1]=qArm[1];  
  qArmDq[2]=qArm[2]+dq;  
  THOROP_kinematics_calculate_arm_torque(dummy, b_matrix[2], rpyangle,qArmDq);
  qArmDq[2]=qArm[2];  
  qArmDq[3]=qArm[3]+dq;  
  THOROP_kinematics_calculate_arm_torque(dummy, b_matrix[3], rpyangle,qArmDq);
  qArmDq[3]=qArm[3];  
  qArmDq[4]=qArm[4]+dq;  
  THOROP_kinematics_calculate_arm_torque(dummy, b_matrix[4], rpyangle,qArmDq);
  qArmDq[4]=qArm[4];  
  qArmDq[5]=qArm[5]+dq;  
  THOROP_kinematics_calculate_arm_torque(dummy, b_matrix[5], rpyangle,qArmDq);
  qArmDq[5]=qArm[5];  
  qArmDq[6]=qArm[6]+dq;  
  THOROP_kinematics_calculate_arm_torque(dummy, b_matrix[6], rpyangle,qArmDq);
  qArmDq[6]=qArm[6];  

  for(i=0;i<7;i++) for (j=0;j<49;j++)
    b_matrix[i][j]=(b_matrix[i][j]-b_matrix0[j])/dq;
/*
  printf("B2 matrix:\n");
  for(i=0;i<7;i++){ 
    for (j=0;j<7;j++)
      printf("%.4f ",b_matrix[2][i*7+j]);
    printf("\n");
  }
  printf("\n");
*/


  for(k=0;k<7;k++){
    for(j=0;j<7;j++){
      //linear term
      acc_torque[k]+=b_matrix0[k*7+j]*qArmAcc[j];

      //quadratic term
      for(i=0;i<7;i++){
        acc_torque2[k]+=(b_matrix[i][k*7+j]-0.5*b_matrix[k][i*7+j]) *qArmVel[i]*qArmVel[j];
      }
    }
  }
/*
  printf("Acc torque:");
  for(k=0;k<7;k++){printf("%.5f ",acc_torque[k]);}
  printf("\n");
  printf("Acc2 torque:");
  for(k=0;k<7;k++){printf("%.5f ",acc_torque2[k]);}
  printf("\n");
*/
}




void THOROP_kinematics_calculate_arm_torque(
  double* stall_torque, double* b_matrix,
  const double *rpyangle,const double *qArm
  ){

  Transform 
    COM0,COM1,COM2,COM3,COM4,COM5,COM6,

    Jac00,
    Jac10,Jac11,
    Jac20,Jac21,Jac22,
    Jac30,Jac31,Jac32,Jac33,
    Jac40,Jac41,Jac42,Jac43,Jac44,
    Jac50,Jac51,Jac52,Jac53,Jac54,Jac55,
    Jac60,Jac61,Jac62,Jac63,Jac64,Jac65,Jac66;
  int i;
  for (i=0;i<7;i++) stall_torque[i]=0;
  for (i=0;i<49;i++) b_matrix[i]=0;

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
  J0.calculate7(COM0,Jac00,JacZZ,JacZZ,JacZZ,JacZZ,JacZZ,JacZZ, MassArm[0],InertiaArm[0]);  
  J1.calculate7(COM1,Jac10,Jac11,JacZZ,JacZZ,JacZZ,JacZZ,JacZZ, MassArm[1],InertiaArm[1]);
  J2.calculate7(COM2,Jac20,Jac21,Jac22,JacZZ,JacZZ,JacZZ,JacZZ, MassArm[2],InertiaArm[2]);  
  J3.calculate7(COM3,Jac30,Jac31,Jac32,Jac33,JacZZ,JacZZ,JacZZ, MassArm[3],InertiaArm[3]);
  J4.calculate7(COM4,Jac40,Jac41,Jac42,Jac43,Jac44,JacZZ,JacZZ, MassArm[4],InertiaArm[4]);  
  J5.calculate7(COM5,Jac50,Jac51,Jac52,Jac53,Jac54,Jac55,JacZZ, MassArm[5],InertiaArm[5]);
  J6.calculate7(COM6,Jac60,Jac61,Jac62,Jac63,Jac64,Jac65,Jac66, MassArm[6],InertiaArm[6]);
  
  J0.accumulate_stall_torque(stall_torque, 0.0,0.0,MassArm[0]*g);
  J1.accumulate_stall_torque(stall_torque, 0.0,0.0,MassArm[1]*g);
  J2.accumulate_stall_torque(stall_torque, 0.0,0.0,MassArm[2]*g);
  J3.accumulate_stall_torque(stall_torque, 0.0,0.0,MassArm[3]*g);
  J4.accumulate_stall_torque(stall_torque, 0.0,0.0,MassArm[4]*g);
  J5.accumulate_stall_torque(stall_torque, 0.0,0.0,MassArm[5]*g);
  J6.accumulate_stall_torque(stall_torque, 0.0,0.0,MassArm[6]*g);  

  J0.dump_b_matrix(b_matrix);
  J1.dump_b_matrix(b_matrix);
  J2.dump_b_matrix(b_matrix);
  J3.dump_b_matrix(b_matrix);
  J4.dump_b_matrix(b_matrix);
  J5.dump_b_matrix(b_matrix);
  J6.dump_b_matrix(b_matrix);
}



void THOROP_kinematics_calculate_leg_torque(
  double* stall_torque,double* b_matrx,
  const double *rpyangle,const double *qLeg,
  int isLeft, double grf, const double *support){

/*
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

  for (int i=0;i<6;i++) {stall_torque[i]=0;acc_torque[i]=0;}
    
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
       .rotateX(qLeg[5]);
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

  COMS = trcopy(COM5).translate(support[0],support[1],0).translate(legLink[6]);
  JacS0 = trcopy(Jac50).translate(support[0],support[1],0).translate(legLink[6]);
  JacS1 = trcopy(Jac51).translate(support[0],support[1],0).translate(legLink[6]);
  JacS2 = trcopy(Jac52).translate(support[0],support[1],0).translate(legLink[6]);
  JacS3 = trcopy(Jac53).translate(support[0],support[1],0).translate(legLink[6]);
  JacS4 = trcopy(Jac54).translate(support[0],support[1],0).translate(legLink[6]);
  JacS5 = trcopy(Jac55).translate(support[0],support[1],0).translate(legLink[6]);

  COM5.translate(legCom[index+5]);
  Jac50.translate(legCom[index+5]);
  Jac51.translate(legCom[index+5]);
  Jac52.translate(legCom[index+5]);
  Jac53.translate(legCom[index+5]);
  Jac54.translate(legCom[index+5]);
  Jac55.translate(legCom[index+5]);

  Transform JacZZ;
  Jacobian J0,J1,J2,J3,J4,J5,JS;

  J0.calculate6(COM0,Jac00,JacZZ,JacZZ,JacZZ,JacZZ,JacZZ,MassLeg[0],InertiaLeg[index+0]);  
  J1.calculate6(COM1,Jac10,Jac11,JacZZ,JacZZ,JacZZ,JacZZ,MassLeg[1],InertiaLeg[index+1]);
  J2.calculate6(COM2,Jac20,Jac21,Jac22,JacZZ,JacZZ,JacZZ,MassLeg[2],InertiaLeg[index+2]);  
  J3.calculate6(COM3,Jac30,Jac31,Jac32,Jac33,JacZZ,JacZZ,MassLeg[3],InertiaLeg[index+3]);
  J4.calculate6(COM4,Jac40,Jac41,Jac42,Jac43,Jac44,JacZZ,MassLeg[4],InertiaLeg[index+4]);  
  J5.calculate6(COM5,Jac50,Jac51,Jac52,Jac53,Jac54,Jac55,MassLeg[5],InertiaLeg[index+5]);
  J6.calculate6(COM5,Jac50,Jac51,Jac52,Jac53,Jac54,Jac55,0,InertiaLeg[index+0]);
  
  J0.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[0]*g);
  J1.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[1]*g);
  J2.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[2]*g);
  J3.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[3]*g);
  J4.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[4]*g);
  JS.accumulate_stall_torque(stall_torque, 0.0,0.0,-grf);
   
*/ 
}




void THOROP_kinematics_calculate_support_leg_torque(
  double* stall_torque,double* b_matrx,
  const double *rpyangle,const double *qLeg,
  int isLeft, double grf, const double *comUpperBody){
/*
  int index = 6;
  const double* legLink0=rlegLink0;

  if (isLeft>0) {
    index = 0;
    legLink0=llegLink0;
  }

  Transform 
    COM0,COM1,COM2,COM3,COM4,COM5,COMB,
    Jac45,
    Jac34,Jac35,
    Jac23,Jac24,Jac25,
    Jac12,Jac13,Jac14,Jac15,
    Jac01,Jac02,Jac03,Jac04,Jac05,
    JacB0,JacB1,JacB2,JacB3,JacB4,JacB5;

  for (int i=0;i<6;i++) {stall_torque[i]=0;acc_torque[i]=0;}
  
  //assumption: foot is on ground with zero angles

  COM5.translateNeg(legLink[6]).translate(legCom[index+5]);

  COM4.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).translate(legCom[index+4]);
  Jac45.translateNeg(legLink[6]).rotateDotXNeg(-qLeg[5])
       .translateNeg(legLink[5]).translate(legCom[index+4]);

  COM3.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).translate(legCom[index+3]);
  Jac34.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateDotYNeg(-qLeg[4])
      .translateNeg(legLink[4]).translate(legCom[index+3]);
  Jac35.translateNeg(legLink[6]).rotateDotXNeg(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).translate(legCom[index+3]);

  COM2.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).translate(legCom[index+2]);
  Jac23.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateDotYNeg(-qLeg[3])
      .translateNeg(legLink[3]).translate(legCom[index+2]);
  Jac24.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateDotYNeg(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).translate(legCom[index+2]);
  Jac25.translateNeg(legLink[6]).rotateDotXNeg(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).translate(legCom[index+2]);

  COM1.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).translate(legCom[index+1]);
  Jac12.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateDotYNeg(-qLeg[2])
      .translateNeg(legLink[2]).translate(legCom[index+1]);
  Jac13.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateDotYNeg(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).translate(legCom[index+1]);
  Jac14.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateDotYNeg(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).translate(legCom[index+1]);
  Jac15.translateNeg(legLink[6]).rotateDotXNeg(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).translate(legCom[index+1]);

  COM0.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).rotateX(-qLeg[1])
      .translateNeg(legLink[1]).translate(legCom[index+0]);
  Jac01.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).rotateDotXNeg(-qLeg[1])
      .translateNeg(legLink[1]).translate(legCom[index+0]);
  Jac02.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateDotYNeg(-qLeg[2])
      .translateNeg(legLink[2]).rotateX(-qLeg[1])
      .translateNeg(legLink[1]).translate(legCom[index+0]);
  Jac03.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateDotYNeg(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).rotateX(-qLeg[1])
      .translateNeg(legLink[1]).translate(legCom[index+0]);
  Jac04.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateDotYNeg(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).rotateX(-qLeg[1])
      .translateNeg(legLink[1]).translate(legCom[index+0]);
  Jac05.translateNeg(legLink[6]).rotateDotXNeg(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).rotateX(-qLeg[1])
      .translateNeg(legLink[1]).translate(legCom[index+0]);       

  COMB.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).rotateX(-qLeg[1])
      .translateNeg(legLink[1]).rotateZ(-qLeg[0])
      .translateNeg(legLink0).translate(comUpperBody);
  JacB0.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).rotateX(-qLeg[1])
      .translateNeg(legLink[1]).rotateDotZNeg(-qLeg[0])
      .translateNeg(legLink0).translate(comUpperBody);
  JacB1.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).rotateDotXNeg(-qLeg[1])
      .translateNeg(legLink[1]).rotateZ(-qLeg[0])
      .translateNeg(legLink0).translate(comUpperBody);
  JacB2.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateDotYNeg(-qLeg[2])
      .translateNeg(legLink[2]).rotateX(-qLeg[1])
      .translateNeg(legLink[1]).rotateZ(-qLeg[0])
      .translateNeg(legLink0).translate(comUpperBody);
  JacB3.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateDotYNeg(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).rotateX(-qLeg[1])
      .translateNeg(legLink[1]).rotateZ(-qLeg[0])
      .translateNeg(legLink0).translate(comUpperBody);
  JacB4.translateNeg(legLink[6]).rotateX(-qLeg[5])
      .translateNeg(legLink[5]).rotateDotYNeg(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).rotateX(-qLeg[1])
      .translateNeg(legLink[1]).rotateZ(-qLeg[0])
      .translateNeg(legLink0).translate(comUpperBody);
  JacB5.translateNeg(legLink[6]).rotateDotXNeg(-qLeg[5])
      .translateNeg(legLink[5]).rotateY(-qLeg[4])
      .translateNeg(legLink[4]).rotateY(-qLeg[3])
      .translateNeg(legLink[3]).rotateY(-qLeg[2])
      .translateNeg(legLink[2]).rotateX(-qLeg[1])
      .translateNeg(legLink[1]).rotateZ(-qLeg[0])
      .translateNeg(legLink0).translate(comUpperBody);

  Transform JacZZ;
  Jacobian JB,J0,J1,J2,J3,J4;

  JB.calculate6(COMB,JacB0,JacB1,JacB2,JacB3,JacB4,JacB5);
  J0.calculate6(COM0,JacZZ,Jac01,Jac02,Jac03,Jac04,Jac05);  
  J1.calculate6(COM0,JacZZ,JacZZ,Jac12,Jac13,Jac14,Jac15);  
  J2.calculate6(COM0,JacZZ,JacZZ,JacZZ,Jac23,Jac24,Jac25);  
  J3.calculate6(COM0,JacZZ,JacZZ,JacZZ,JacZZ,Jac34,Jac35);  
  J4.calculate6(COM0,JacZZ,JacZZ,JacZZ,JacZZ,JacZZ,Jac35);  
  

  double net_grf = grf-(MassLeg[0]+MassLeg[1]+MassLeg[2]+MassLeg[3]+MassLeg[4]+MassLeg[5])*g;

  JB.accumulate_stall_torque(stall_torque, 0.0,0.0,net_grf);
  J0.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[0]*g);
  J1.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[1]*g);
  J2.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[2]*g);
  J3.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[3]*g);
  J4.accumulate_stall_torque(stall_torque, 0.0,0.0,MassLeg[4]*g);
  
 
  J0.accumulate_acc_torque(acc_torque, &qLegAcc[0], MassLeg[0], &InertiaLeg[index+0][0]);
  J1.accumulate_acc_torque(acc_torque, &qLegAcc[0], MassLeg[1], &InertiaLeg[index+1][0]);
  J2.accumulate_acc_torque(acc_torque, &qLegAcc[0], MassLeg[2], &InertiaLeg[index+2][0]);
  J3.accumulate_acc_torque(acc_torque, &qLegAcc[0], MassLeg[3], &InertiaLeg[index+3][0]);
  J4.accumulate_acc_torque(acc_torque, &qLegAcc[0], MassLeg[4], &InertiaLeg[index+4][0]);
  J5.accumulate_acc_torque(acc_torque, &qLegAcc[0], MassLeg[5], &InertiaLeg[index+5][0]);
*/ 
}



std::vector<double> THOROP_kinematics_calculate_arm_velocity(  
  const double *qArm, const double *pVelArm, 
  const double *qWaist, const double *rpyangle, 
  double handx, double handy, double handz, int is_left){

  Transform 
    COM,Jac0,Jac1,Jac2,Jac3,Jac4,Jac5,Jac6;


  Transform torso;
  torso.rotateX(rpyangle[0]).rotateY(rpyangle[1])
      .rotateZ(qWaist[0]).rotateY(qWaist[1]);

    

  COM= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(handx,handy,handz);

  Jac0= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(handx,handy,handz);

  Jac1= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(handx,handy,handz);

  Jac2= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(handx,handy,handz);

  Jac3= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(handx,handy,handz);

  Jac4= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(handx,handy,handz);

  Jac5= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(handx,handy,handz);

  Jac6= trcopy(torso).rotateY(qArm[0]).translate(armLink[1])
        .rotateZ(qArm[1]).translate(armLink[2])
        .rotateX(qArm[2]).translate(armLink[3])
        .rotateY(qArm[3]).translate(armLink[4])
        .rotateX(qArm[4]).translate(armLink[5])
        .rotateZ(qArm[5]).translate(armLink[6])
        .rotateX(qArm[6]).translate(handx,handy,handz);

  Jacobian J;
  J.calculateVel7(COM,Jac0,Jac1,Jac2,Jac3,Jac4,Jac5,Jac6);    
  //now we have 6x7 jacobian matrix for the end effector


//TODO


  //return joint angles
  std::vector<double> qVelArm(7);
  return qVelArm;
}



