#ifndef EPOS_SLAVE_H
#define EPOS_SLAVE_H

/*
 * dsp402_slave : CANopen communication interface for EPOS2 motor controllers 
 * author : Mike Hopkins
 */

#include "dsp402_slave.h"

/* defines standard epos object dictionary entries */
#define EPOS_NODE_ID                                      0x2000
#define EPOS_CAN_BITRATE                                  0x2001
#define EPOS_RS232_BAUDRATE                               0x2002
#define EPOS_RS232_FRAME_TIMEOUT                          0x2005
#define EPOS_USB_FRAME_TIMEOUT                            0x2006
#define EPOS_MISCELLANEOUS_CONFIGURATION                  0x2008
#define EPOS_CAN_BITRATE_DISPLAY                          0x200A
#define EPOS_INCREMENTAL_ENCODER_1_COUNTER                0x2020
#define EPOS_INCREMENTAL_ENCODER_1_COUNTER_AT_INDEX_PULSE 0x2021
#define EPOS_HALL_SENSOR_PATTERN                          0x2022
#define EPOS_CURRENT_ACTUAL_VALUE_AVERAGED                0x2027
#define EPOS_VELOCITY_ACTUAL_VALUE_AVERAGED               0x2028
#define EPOS_AUXILIARY_VELOCITY_ACTUAL_VALUE_AVERAGED     0x2029
#define EPOS_CURRENT_MODE_SETTING_VALUE                   0x2030
#define EPOS_CURRENT_DEMAND_VALUE                         0x2031
#define EPOS_POSITION_MODE_SETTING_VALUE                  0x2062
#define EPOS_AUXILIARY_VELOCITY_SENSOR_ACTUAL_VALUE       0x2069
#define EPOS_VELOCITY_MODE_SETTING_VALUE                  0x206B
#define EPOS_AUXILIARY_VELOCITY_ACTUAL_VALUE              0x206C
#define EPOS_CONFIGURATION_OF_DIGITAL_INPUTS              0x2070
#define EPOS_DIGITAL_INPUT_FUNCTIONALITIES                0x2071
#define EPOS_DIGITAL_OUTPUT_FUNCTIONALITIES               0x2078
#define EPOS_CONFIGURATION_OF_DIGITAL_OUTPUTS             0x2079
#define EPOS_CONFIGURATION_OF_ANALOG_INPUTS               0x207B
#define EPOS_ANALOG_INPUTS                                0x207C
#define EPOS_ANALOG_INPUT_FUNCTIONALITIES_EXECUTION_MASK  0x207D
#define EPOS_ANALOG_OUTPUT_1                              0x207E
#define EPOS_CURRENT_THRESHOLD_FOR_HOMING_MODE            0x2080 
#define EPOS_HOME_POSITION                                0x2081
#define EPOS_FOLLOWING_ERROR_ACTUAL_VALUE                 0x20F4
#define EPOS_HOLDING_BRAKE_CONFIGURATION                  0x2100
#define EPOS_STANDSTILL_WINDOW_CONFIGURATION              0x2101
#define EPOS_SENSOR_CONFIGURATION                         0x2210
#define EPOS_SSI_ENCODER_CONFIGURATION                    0x2211
#define EPOS_INCREMENTAL_ENCODER_2_CONFIGURATION          0x2212
#define EPOS_SINUS_INCREMENTAL_ENCODER_2_CONFIGURATION    0x2213
#define EPOS_CONTROLLER_STRUCTURE                         0x2220
#define EPOS_GEAR_CONFIGURATION                           0x2230
#define EPOS_DIGITAL_POSITION_INPUT                       0x2300
#define EPOS_ANALOG_CURRENT_SETPOINT_CONFIGURATION        0x2301
#define EPOS_ANALOG_VELOCITY_SETPOINT_CONFIGURATION       0x2302
#define EPOS_ANALOG_POSITION_SETPOINT_CONFIGURATION       0x2303

class epos_slave : public dsp402_slave {
public:
  epos_slave(uint8_t node_id = 1);
};

#endif
