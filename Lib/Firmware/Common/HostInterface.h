#ifndef HOST_INTERFACE_H
#define HOST_INTERFACE_H

#include <inttypes.h>
#include "DynamixelPacket.h"

int HostInit();

//read off data from the circular buffer and see
//whether a complete packet came in.
int HostReceivePacket(DynamixelPacket * packet);

//package data and send to the host
int HostSendPacket(uint8_t id, uint8_t type, uint8_t * buf, uint8_t size);

//send the raw packet to the host
//assumes the data is already in the packet format
int HostSendRawPacket(DynamixelPacket * packet);

//send raw data
int HostSendRawData(uint8_t * data, uint8_t size);

#endif //HOST_INTERFACE_H
