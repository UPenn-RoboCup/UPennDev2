#include "config.h"
#include "GpsInterface.h"
#include "MagicMicroCom.h"

int GpsInit()
{
  GPS_COM_PORT_INIT();
  GPS_COM_PORT_SETBAUD(GPS_BAUD_RATE);

  return 0;
}

int GpsReceiveLine(uint8_t ** buf)
{
  static uint8_t buffer[MMC_GPS_LINE_SIZE_MAX];
  static int lenReceived = 0;  //number of chars received so far
  static uint8_t * bp = NULL;  //pointer to the next write position
  
  int ret = -1;
  int c;
  *buf = NULL;                 //reset the output pointer
  
  c = GPS_COM_PORT_GETCHAR();         //read one char (non-blocking)
  while(c != EOF)              //if got char, process
  {
    lenReceived++;
    
    switch (lenReceived)
    {
      case 1:                    //new line
      
        if ( c != '\n' )  //make sure it's not end of line
          bp          = buffer;        //reset the pointer
        else
        {
          lenReceived = 0;
          continue;
        }
      
      //fall through
      default:
    
        if ( c == '\n' )  //see if we are at end of line
        {
          //store, reset, assign output pointer and return
          ret         = lenReceived;
          lenReceived = 0;
          *bp         = c;   //store c, since we are about to return
          *buf        = buffer;
          return ret;
        }
      
        //not end of line
        if (lenReceived == MMC_GPS_LINE_SIZE_MAX)
        {
          //reset if about to overflow (we should not get this far..)
          lenReceived = 0;
        }
    }
    
    *bp++ = c;            //store the char after processing
    c=GPS_COM_PORT_GETCHAR();
  }
  
  return ret;
}
