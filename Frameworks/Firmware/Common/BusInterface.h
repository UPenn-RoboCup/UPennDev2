#ifndef BUS_INTERFACE_H
#define BUS_INTERFACE_H

#include <inttypes.h>
#include "MagicMicroCom.h"
#include "DynamixelPacket.h"

int BusInit();

//read off data from the circular buffer and see
//whether a complete packet came in.
int BusReceivePacket(DynamixelPacket * packet);

//package data and send to the host
int BusSendPacket(uint8_t id, uint8_t type, uint8_t * buf, uint8_t size);

//send the raw packet to the host
//assumes the data is already in the packet format
int BusSendRawPacket(DynamixelPacket * packet);

int BusSendRawData(uint8_t * data, uint8_t size);

#endif //BUS_INTERFACE_H
