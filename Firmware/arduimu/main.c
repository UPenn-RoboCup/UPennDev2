#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>


#include "uart.h"
#include "adc.h"
#include "config.h"
#include "DynamixelPacket.h"

#define DYN_IMU_DEVICE_ID 2
#define PKT_TYPE_IMU_ROT a
enum { PKT_TYPE_IMU_RAW,
  PKT_TYPE_IMU_FILTERED,
  PKT_TYPE_IMU_ROT,
  PKT_TYPE_MAG_RAW,
  PKT_TYPE_IMU_RESET,
  PKT_TYPE_IMU_FILTERED2
};
enum { OUTPUT_MODE_BINARY,
  OUTPUT_MODE_TEXT
};

volatile uint16_t adcVals[NUM_ADC_CHANNELS];

int ImuPacketHandler();
void ResetImu();
extern volatile float roll, pitch, yaw;

uint8_t outputMode = OUTPUT_MODE_TEXT; 

void init(void)
{
  uart_init();
  uart_setbaud(HOST_BAUD_RATE);

  //LEDS
  DDRD |= _BV(PD5) | _BV(PD6) | _BV(PD7);
  PORTD |= _BV(PD5) | _BV(PD6) | _BV(PD7);

  //ADC
  adc_init();

  ResetImu();

  sei();
}



int main(void)
{
  //int c;
  int jk;
  int ret,ret2,len;
  uint16_t cntr=0;
  uint16_t t0;
  uint16_t dt;
  uint8_t ledCntr=0;

  const int bufSize = 64;
  uint8_t buf1[bufSize];
  uint8_t buf2[bufSize];
  float * fbuf1;

  init();

  while (1) 
  {
    cli();   //disable interrupts to prevent race conditions while copying
    ret = adc_get_data();
    sei();   //re-enable interrupts


    if (ret > 0)
    {
      t0 = TCNT1;
      ret2 = ImuPacketHandler();
      dt = TCNT1-t0;

      if (outputMode == OUTPUT_MODE_TEXT)
      {
        if ( ret2 == 0)
        {
          //uart_printf("(%d %d %d) %d\r\n",(int)(roll*180.0/M_PI),(int)(pitch*180.0/M_PI),(int)(yaw*180.0/M_PI),dt);
          uart_printf("%d %d %d\n",
              (int)(roll*180.0/M_PI), (int)(pitch*180.0/M_PI), (int)(yaw*180.0/M_PI)
              );
          cntr++;
        }
        else if (ret2 == 1)
        {
					// uart_printf("(%d %d %d)   (%d %d %d)  %d\r\n",
          /*
          uart_printf("%d %d %d %d %d %d %d\n",
              adcVals[ADC_AX_IND],adcVals[ADC_AY_IND],adcVals[ADC_AZ_IND],
              adcVals[ADC_WX_IND],adcVals[ADC_WY_IND],adcVals[ADC_WZ_IND], dt);
          */
          uart_printf("%d %d %d %d %d %d %d %d %d %d\n",
              adcVals[ADC_AX_IND],adcVals[ADC_AY_IND],adcVals[ADC_AZ_IND],
              adcVals[ADC_WX_IND],adcVals[ADC_WY_IND],adcVals[ADC_WZ_IND], 
              (int)(roll*180.0/M_PI), (int)(pitch*180.0/M_PI), (int)(yaw*180.0/M_PI),
              dt);
        }
      }
      else if (outputMode == OUTPUT_MODE_BINARY)
      {
        if ( ret2 == 0)
        {
          cntr++;
          fbuf1 = (float*)buf1;
          *((uint32_t*)fbuf1++) = cntr;
          *fbuf1++ = roll;
          *fbuf1++ = pitch;
          *fbuf1++ = yaw;

          len = DynamixelPacketWrapData( DYN_IMU_DEVICE_ID,PKT_TYPE_IMU_ROT,buf1,16,buf2,bufSize);
          if (len > 0) {
            for(jk=0;jk<len;jk++)
              uart_putchar( buf2[jk] );
          }
        }
      }

      if ((ret2==0) && (cntr % 7 == 0))
      {
        ledCntr++;

        switch (ledCntr)
        {
          case 1:
            PORTD &= ~(_BV(PD5) | _BV(PD6) | _BV(PD7));
            PORTD |= _BV(PD5);
            break;
          case 2:
            PORTD &= ~(_BV(PD5) | _BV(PD6) | _BV(PD7));
            PORTD |= _BV(PD6);
            break;
          case 3:
            PORTD &= ~(_BV(PD5) | _BV(PD6) | _BV(PD7));
            PORTD |= _BV(PD7);
            break;
          default:
            PORTD &= ~(_BV(PD5) | _BV(PD6) | _BV(PD7));
            ledCntr=0;
            break;
        }
      }
    }
  }

  /*
     if (len > 0)
     {
     uart_printf("%d %d %d %d %d %d\r\n",adcVals[ADC_AX_IND],adcVals[ADC_AY_IND],adcVals[ADC_AZ_IND],
     adcVals[ADC_WX_IND],adcVals[ADC_WY_IND],adcVals[ADC_WZ_IND] );

     }    

     c = uart_getchar();
     if(c != EOF)
     {
     uart_putchar(c);
     PORTD ^= _BV(PD6);
     }
     }
     */  
return 0;
}

