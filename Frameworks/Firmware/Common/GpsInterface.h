#ifndef GPS_INTERFACE_H
#define GPS_INTERFACE_H

#include <inttypes.h>

int GpsInit();

//read off data from the circular buffer and see
//whether a complete packet came in.
//packet is complete upon receiving an end of line
int GpsReceiveLine(uint8_t ** buf);


#endif //GPS_INTERFACE_H
