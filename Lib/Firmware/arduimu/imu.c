#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "config.h"

volatile extern uint16_t adcVals[NUM_ADC_CHANNELS];

//latest rotation matrix
volatile float R[9] = {1,0,0, 0,1,0, 0,0,1};

//the delta rotation matrix
volatile float dR[9] = {1,0,0, 0,1,0, 0,0,1};

//actual accelerations and angular rates
volatile float ax,ay,az,wx,wy,wz;
volatile float axa,aya,aza,wxa,wya,wza;

//acceleration biases
volatile float axb = BIAS_ACC_X;
volatile float ayb = BIAS_ACC_Y;
volatile float azb = BIAS_ACC_Z;

//rate gyro biases (to be calibrated at start up)
volatile float wxb = 0;
volatile float wyb = 0;
volatile float wzb = 0;

volatile float roll,pitch,yaw;

/*
volatile float wxa = 0;
volatile float wya = 0;
volatile float wza = 0;
*/

//error in biases (experimental)
//float wxdb = 0;
//float wydb = 0;
//float wzdb = 0;

//error between rate gyro and accelerometer predictions (g for global, b for body frame)
/*
float erollg  = 0;
float epitchg = 0;
float eyawg   = 0;
float erollb  = 0;
float epitchb = 0;
float eyawb   = 0;
*/

volatile float RR21;
volatile float RR22;
volatile float RR20;
volatile float RR10;
volatile float RR00;

//sensitivities for converting raw values into real units
const float axs = SENS_ACC_X;
const float ays = SENS_ACC_Y;
const float azs = SENS_ACC_Z;
const float wxs = ADC_VOLTAGE_MV/1023.0 * SENS_GYRO_X;
const float wys = ADC_VOLTAGE_MV/1023.0 * SENS_GYRO_Y;
const float wzs = ADC_VOLTAGE_MV/1023.0 * SENS_GYRO_Z;

const float dt=64.0*ADC_TIMER_PERIOD_TICS/F_CPU; //IMU_SAMPLE_RATE;

//factors for combining rate gyro and accelerometer info (malfa=1-alpha)
float alpha=0.005;
float malpha = 0.995;


float groll,gpitch,gyaw,aroll,apitch,nroll,npitch,nyaw,myaw;
float sinx,siny,sinz,cosx,cosy,cosz;
float wxt,wyt,wzt;


volatile uint16_t wxcMin = 1023;
volatile uint16_t wxcMax = 0;
volatile uint16_t wycMin = 1023;
volatile uint16_t wycMax = 0;
volatile uint16_t wzcMin = 1023;
volatile uint16_t wzcMax = 0;


uint16_t t;
uint16_t imuUpdateCntr=0;
uint8_t gyrosCalibrated = 0;
volatile float outVals[15];

volatile int filterLength = FILTER_LENGTH;


void ResetR()
{
  R[0] = 1; R[1] = 0; R[2] = 0; R[3] = 0;
  R[4] = 1; R[5] = 0; R[6] = 0; R[7] = 0;
  R[8] = 1;
}

void ResetdR()
{
  dR[0] = 1; dR[1] = 0; dR[2] = 0; dR[3] = 0;
  dR[4] = 1; dR[5] = 0; dR[6] = 0; dR[7] = 0;
  dR[8] = 1;
}

void ResetImu()
{
  ResetR();
  ResetdR();
  
  imuUpdateCntr=0;
  gyrosCalibrated=0;

  wxb = 0;
  wyb = 0;
  wzb = 0;
  
  wxa = 0;
  wya = 0;
  wza = 0;
  
  axa = 0;
  aya = 0;
  aza = 0;

  wxcMin = 1023;
  wxcMax = 0;
  wycMin = 1023;
  wycMax = 0;
  wzcMin = 1023;
  wzcMax = 0;
}

//this will be called when last channel has been read
int ImuPacketHandler()
{
  imuUpdateCntr++;
  
  //initialization of rate gyros
  if (!gyrosCalibrated)
  {
  
    //accumulate values
    if (imuUpdateCntr <= NUM_GYRO_CALIB_SAMPLES)
    {
      wxb+= (int16_t)adcVals[ADC_WX_IND];
      wyb+= (int16_t)adcVals[ADC_WY_IND];
      wzb+= (int16_t)adcVals[ADC_WZ_IND];
      
      if (adcVals[ADC_WX_IND] < wxcMin)
        wxcMin = adcVals[ADC_WX_IND];

      if (adcVals[ADC_WY_IND] < wycMin)
        wycMin = adcVals[ADC_WY_IND];

      if (adcVals[ADC_WZ_IND] < wzcMin)
        wzcMin = adcVals[ADC_WZ_IND];

      if (adcVals[ADC_WX_IND] > wxcMax)
        wxcMax = adcVals[ADC_WX_IND];

      if (adcVals[ADC_WY_IND] > wycMax)
        wycMax = adcVals[ADC_WY_IND];

      if (adcVals[ADC_WZ_IND] > wzcMax)
        wzcMax = adcVals[ADC_WZ_IND];
    }
    
    //calculate average
    if (imuUpdateCntr == NUM_GYRO_CALIB_SAMPLES)
    {
      wxb /= NUM_GYRO_CALIB_SAMPLES;
      wyb /= NUM_GYRO_CALIB_SAMPLES;
      wzb /= NUM_GYRO_CALIB_SAMPLES;
      
      
      if (wxb > (NOMINAL_GYRO_BIAS + GYRO_BIAS_MARGIN) ||
          wxb < (NOMINAL_GYRO_BIAS - GYRO_BIAS_MARGIN) ||
          wyb > (NOMINAL_GYRO_BIAS + GYRO_BIAS_MARGIN) ||
          wyb < (NOMINAL_GYRO_BIAS - GYRO_BIAS_MARGIN) ||
          wzb > (NOMINAL_GYRO_BIAS + GYRO_BIAS_MARGIN) ||
          wzb < (NOMINAL_GYRO_BIAS - GYRO_BIAS_MARGIN) ||
          (wxcMax - wxcMin) > MAX_GYRO_CALIB_NOSE ||
          (wycMax - wycMin) > MAX_GYRO_CALIB_NOSE ||
          (wzcMax - wzcMin) > MAX_GYRO_CALIB_NOSE  )
      {
        //sometime bad happened during gyro calibration, so reset
        ResetImu();
        return 1;
      }
      
      gyrosCalibrated = 1;
    }
    return 1;
  }
 
  //accumulate values
  ax =  ((float)adcVals[ADC_AX_IND] - axb) * axs;
  ay =  ((float)adcVals[ADC_AY_IND] - ayb) * ays;
  az =  ((float)adcVals[ADC_AZ_IND] - azb) * azs;
  wx =  -((float)adcVals[ADC_WX_IND] - wxb) * wxs;
  wy =  -((float)adcVals[ADC_WY_IND] - wyb) * wys;
  wz =  ((float)adcVals[ADC_WZ_IND] - wzb) * wzs;
  

/*  
#define FLIP
#ifdef FLIP
  ay = -ay;
  az = -az;
  wy = -wy;
  wz = -wz;
#endif
*/  
  
  wxt = wx*dt;
  wyt = wy*dt;
  wzt = wz*dt;
  
  axa += ax;
  aya += ay;
  aza += az;
  wxa += wx;
  wya += wy;
  wza += wz;
  
  //stuff the delta matrix. This is derived from linearized equation
  //dR = dRz * dRy * dRx, with sinx = x and cosx = 1, since dt is small
  //only part of the dR matrix is calculated, since we don't need all 
  //components in the CalcGyroRPY function
  dR[1] += wzt;
  dR[2] += -wyt;
  dR[3] += -wzt;
  dR[5] += wxt;
  dR[6] += wyt;
  dR[7] += -wxt;
  
  //skip processing every other time
  if ((imuUpdateCntr % filterLength) != 0)
    return 2;
    
  //calculate the gyro update
  //--------------------------
  float RR21 = R[2]*dR[3] + R[5]       + R[8]*dR[5];
  float RR22 = R[2]*dR[6] + R[5]*dR[7] + R[8]      ;
  float RR20 = R[2]       + R[5]*dR[1] + R[8]*dR[2];
  float RR10 = R[1]       + R[4]*dR[1] + R[7]*dR[2]; 
  float RR00 = R[0]       + R[3]*dR[1] + R[6]*dR[2];

  //extract the values after rate gyro update
  groll  = atan2(RR21,RR22);
  gpitch = atan2(-RR20,sqrt(RR21*RR21 + RR22*RR22));
  gyaw   = atan2(RR10,RR00);
  
  //zero out the delta matrix, since the values in dR can be accumulated
  //over several data collections
  ResetdR();
  //---------------------------
    
  //calculate the roll and pitch based on the current accelerometer values
  apitch  = -atan2(ax,sqrt(ay*ay + az*az));
  aroll   =  atan2(ay,az);
  
  //error between the gyro updated roll and pitch and the accelerometer predictions
  //erollg  = aroll  - groll;
  //epitchg = apitch - gpitch;

  //combine the rate gyro prediction with accelerometer prediction
  nroll   = alpha*aroll  + malpha*groll;
  npitch  = alpha*apitch + malpha*gpitch;
  nyaw    = gyaw; //no accelerometer prediction for yaw
  
  sinx    = sin(nroll);
  siny    = sin(npitch);
  cosx    = cos(nroll);
  cosy    = cos(npitch);
  
  //calculate the components of the rotation matrix for the next iteration
  sinz = sin(nyaw);
  cosz = cos(nyaw);
  
  float sinysinx = siny*sinx;
  float sinycosx = siny*cosx;
  R[0] = cosz*cosy;
  R[1] = sinz*cosy;
  R[2] = -siny;
  R[3] = cosz*sinysinx - sinz*cosx;
  R[4] = sinz*sinysinx + cosz*cosx;
  R[5] = cosy*sinx;
  R[6] = cosz*sinycosx + sinz*sinx;
  R[7] = sinz*sinycosx - cosz*sinx;
  R[8] = cosy*cosx;
  
  
  //errors in bias (experimental)
  //erollb  = R[0]*erollg + R[1]*epitchg;
  //epitchb = R[3]*erollg + R[4]*epitchg;
  //eyawb   = R[6]*erollg + R[7]*epitchg;
  //wxdb += erollb*0.001;
  //wydb += epitchb*0.001; 
  //wzdb += eyawb*0.005;
  
  roll = nroll;
  pitch = npitch;
  yaw  = nyaw;

  
  
  axa = 0;
  aya = 0;
  aza = 0;
  wxa = 0;
  wya = 0;
  wza = 0;

  return 1;
}

