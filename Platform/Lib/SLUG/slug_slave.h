#ifndef _SLUG_SLAVE_H_
#define _SLUG_SLAVE_H_

/******************************************************************************
 * slug_slave : CANopen communication interface for motor slug
 * authors : Mike Hopkins, Steve Ressler
 ******************************************************************************/

#include "co_slave.h"

/* defines standard epos object dictionary entries */
#define SLUG_NODE_ID                                      0x2000
#define SLUG_CAN_BIT_RATE                                 0x2001
#define SLUG_OPERATING_MODE                               0x2002
#define SLUG_OPERATING_MODE_DISPLAY                       0x2003
#define SLUG_STATUS_CODE                                  0x2004
#define SLUG_ERROR_CODE                                   0x2005

#define SLUG_JOINT_FORCE_CONSTANT                         0x2100
#define SLUG_JOINT_POSITION_CONSTANT                      0x2101
#define SLUG_JOINT_VELOCITY_CONSTANT                      0x2102
#define SLUG_JOINT_PID_GAIN_CONSTANT                      0x2103
#define SLUG_JOINT_FORCE_MAX                              0x2104
#define SLUG_JOINT_POSITION_MAX                           0x2105
#define SLUG_JOINT_POSITION_MIN                           0x2106
#define SLUG_JOINT_VELOCITY_MAX                           0x2107
#define SLUG_JOINT_FORCE_SETPOINT                         0x2108
#define SLUG_JOINT_POSITION_SETPOINT                      0x2109
#define SLUG_JOINT_VELOCITY_SETPOINT                      0x210a
#define SLUG_JOINT_FORCE_ESTIMATE                         0x210b
#define SLUG_JOINT_POSITION_ESTIMATE                      0x210c
#define SLUG_JOINT_VELOCITY_ESTIMATE                      0x210d
#define SLUG_JOINT_POSITION_SENSOR_BIAS                   0x210e
#define SLUG_JOINT_POSITION_P_GAIN                        0x210f
#define SLUG_JOINT_POSITION_I_GAIN                        0x2110
#define SLUG_JOINT_POSITION_D_GAIN                        0x2111
#define SLUG_JOINT_POSITION_D_BREAK_FREQUENCY             0x2112
#define SLUG_JOINT_FORCE_DEMAND                           0x2113

#define SLUG_MOTOR_FORCE_MAX                              0x2200
#define SLUG_MOTOR_CURRENT_MAX                            0x2201
#define SLUG_MOTOR_FORCE_SETPOINT                         0x2202
#define SLUG_MOTOR_FORCE_ESTIMATE                         0x2203
#define SLUG_MOTOR_CURRENT_ESTIMATE                       0x2204
#define SLUG_MOTOR_FORCE_SENSOR_BIAS                      0x2205
#define SLUG_MOTOR_FORCE_FF_CONSTANT                      0x2206
#define SLUG_MOTOR_FORCE_P_GAIN                           0x2207
#define SLUG_MOTOR_FORCE_I_GAIN                           0x2208
#define SLUG_MOTOR_FORCE_D_GAIN                           0x2209
#define SLUG_MOTOR_FORCE_D_BREAK_FREQUENCY                0x220a
#define SLUG_MOTOR_CURRENT_DEMAND                         0x220b

class slug_slave : public co_slave {
public:
  slug_slave(uint8_t node_id = 1);
};

#endif
