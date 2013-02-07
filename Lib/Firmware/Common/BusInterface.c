#include "config.h"
#include "BusInterface.h"
#include "MagicMicroCom.h"

//enable communication to the rs485 bus
int BusInit()
{
  BUS_COM_PORT_INIT();
  BUS_COM_PORT_SETBAUD(BUS_BAUD_RATE);
  
  return 0;
}


//Check if a valid packet has been received
//and return instantanously. This lets buffer
//the packets that have not yet been completely
//received without blocking on receiving routines
//Also verifies the checksum
int BusReceivePacket(DynamixelPacket * packet)
{
  int ret = -1;

  int c = BUS_COM_PORT_GETCHAR();         //read one char (non-blocking)
  while( c != EOF)
  {
    ret = DynamixelPacketProcessChar(c,packet);
    if (ret > 0)
      break;
    c   = BUS_COM_PORT_GETCHAR();
  }

  return ret;
}


int BusSendPacket(uint8_t id, uint8_t type, uint8_t * buf, uint8_t size)
{
  if (size > 254)
    return -1;

  uint8_t size2 = size+2;
  uint8_t ii;
  uint8_t checksum=0;

  BUS_COM_PORT_PUTCHAR(0xFF);   //two header bytes
  BUS_COM_PORT_PUTCHAR(0xFF);
  BUS_COM_PORT_PUTCHAR(id);
  BUS_COM_PORT_PUTCHAR(size2);  //length
  BUS_COM_PORT_PUTCHAR(type);
  
  checksum += id + size2 + type;
  
  //payload
  for (ii=0; ii<size; ii++)
  {
    BUS_COM_PORT_PUTCHAR(*buf);
    checksum += *buf++;
  }
  
  BUS_COM_PORT_PUTCHAR(~checksum);
  
  return 0;
}

int BusSendRawPacket(DynamixelPacket * packet)
{
  uint8_t * buf = packet->buffer;
  uint8_t size  = packet->lenExpected;
  uint8_t ii;
  
  for (ii=0; ii<size; ii++)
    BUS_COM_PORT_PUTCHAR(*buf++);
    
  return 0;
}

int BusSendRawData(uint8_t * data, uint8_t size)
{
  uint8_t ii;

  for (ii=0; ii<size; ii++)
    BUS_COM_PORT_PUTCHAR(*data++);

  return 0;
}

