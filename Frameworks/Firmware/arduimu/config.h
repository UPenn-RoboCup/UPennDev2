#ifndef CONFIG_H
#define CONFIG_H

#include "uart.h"
#include "timer1.h"

#define ADC_TIMER_PRESCALER 64  //set in timerx.c
#define ADC_RATE 100 //Hz
#define FILTER_LENGTH 2
#define ADC_TIMER_PERIOD_TICS F_CPU/ADC_TIMER_PRESCALER/ADC_RATE  //2500 = 100Hz with 1/64 prescaler
#define NUM_ADC_CHANNELS 6

#define ADC_VOLTAGE_MV 3300.0


#define NOMINAL_GYRO_BIAS 370
#define GYRO_BIAS_MARGIN 30

#define MAX_GYRO_CALIB_NOSE 10

//accelerometer biases
#define BIAS_ACC_X 512
#define BIAS_ACC_Y 512
#define BIAS_ACC_Z 512

//accelerometer sensitivities (g/mV)
#define SENS_ACC_X 2.0/890.0
#define SENS_ACC_Y 2.0/850.0
#define SENS_ACC_Z 2.0/835.0

//rate gyro bias (rad/mV)
#define SENS_GYRO_X M_PI/180.0/3.50
#define SENS_GYRO_Y M_PI/180.0/3.55
#define SENS_GYRO_Z M_PI/180.0/3.45


//number of calibration samples for rate gyros
#define NUM_GYRO_CALIB_SAMPLES 100

//mapping of the ADC measurements to appropriate values
#define ADC_AX_IND 0
#define ADC_AY_IND 1
#define ADC_AZ_IND 2
#define ADC_WX_IND 4
#define ADC_WY_IND 5
#define ADC_WZ_IND 3


#define timer1_period_tics ADC_TIMER_PERIOD_TICS

#define DEBUG

//communications with host



#define HOST_BAUD_RATE 115200
#define HOST_COM_PORT_INIT    uart_init
#define HOST_COM_PORT_SETBAUD uart_setbaud
#define HOST_COM_PORT_GETCHAR uart_getchar
#define HOST_COM_PORT_PUTCHAR uart_putchar
#define HOST_COM_PORT_PUTSTR  uart_putstr

#define ADC_TIMER_RESET timer1_reset
#define ADC_TIMER_INIT timer1_init
#define ADC_TIMER_SET_COMPA_CALLBACK timer1_set_compa_callback
#define ADC_TIMER_COMPA timer1_compa

#endif //CONFIG_H
