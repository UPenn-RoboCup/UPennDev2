/*!
  \author Satofumi KAMIMURA

  $Id: open_urg_sensor.c,v d00944669fc8 2011/02/16 13:41:09 Satofumi $
*/

#include "framework/sensor/lidar/open_urg_sensor.h"
#include "framework/sensor/lidar/urg_utils.h"
#include "framework/sensor/lidar/urg_detect_os.h"
#include <string.h>
#include <stdio.h>


int open_urg_sensor(urg_t *urg, char const* ip_addr)
{

    urg_connection_type_t connection_type = URG_ETHERNET;
    long baudrate_or_port = 10940;
    const char *ip_address = ip_addr;
    const char *device = ip_address;

    if (urg_open(urg, connection_type, device, baudrate_or_port) < 0) {
        printf("urg_open: %s, %ld: %s\n",
            device, baudrate_or_port, urg_error(urg));
        return -1;
    }

    return 0;
}
