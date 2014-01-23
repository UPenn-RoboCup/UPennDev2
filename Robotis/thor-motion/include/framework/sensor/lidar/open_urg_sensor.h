#ifndef OPEN_URG_SENSOR_H
#define OPEN_URG_SENSOR_H

/*!
  \author Satofumi KAMIMURA

  $Id$
*/

#include "urg_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

extern int open_urg_sensor(urg_t *urg,  char const* ip_addr);

#ifdef __cplusplus
}
#endif

#endif /* !OPEN_URG_SENSOR_H */
