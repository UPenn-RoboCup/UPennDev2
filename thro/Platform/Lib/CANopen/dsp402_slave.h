#ifndef DSP402_SLAVE_H
#define DSP402_SLAVE_H

/*
 * dsp402_slave : base class for dsp402 compliant slave devices 
 * author : Mike Hopkins
 */

#include "co_slave.h"

/* defines standard dsp402 object dictionary entries */
#define DSP402_ABORT_CONNECTION_OPTION_CODE   0x6007
#define DSP402_ERROR_CODE                     0x603f
#define DSP402_CONTROLWORD                    0x6040
#define DSP402_STATUSWORD                     0x6041
#define DSP402_QUICK_STOP_OPTION_CODE         0x605A
#define DSP402_SHUTDOWN_OPTION_CODE           0x605B
#define DSP402_DISABLE_OPERATION_OPTION_CODE  0x605C
#define DSP402_STOP_OPTION_CODE               0x605D
#define DSP402_FAULT_REACTION_OPTION_CODE     0x605E
#define DSP402_MODES_OF_OPERATION             0x6060
#define DSP402_MODES_OF_OPERATION_DISPLAY     0x6061
#define DSP402_POSITION_DEMAND_VALUE          0x6062
#define DSP402_POSITION_ACTUAL_VALUE_INTERNAL 0x6063
#define DSP402_POSITION_ACTUAL_VALUE          0x6064
#define DSP402_FOLLOWING_ERROR_WINDOW         0x6065
#define DSP402_FOLLOWING_ERROR_TIME_OUT       0x6066
#define DSP402_POSITION_WINDOW                0x6067
#define DSP402_POSITION_WINDOW_TIME           0x6068
#define DSP402_VELOCITY_SENSOR_ACTUAL_VALUE   0x6069
#define DSP402_SENSOR_SELECTION_CODE          0x606A
#define DSP402_VELOCITY_DEMAND_VALUE          0x606B
#define DSP402_VELOCITY_ACTUAL_VALUE          0x606C
#define DSP402_VELOCITY_WINDOW                0x606D
#define DSP402_VELOCITY_WINDOW_TIME           0x606E
#define DSP402_TARGET_TORQUE                  0x6071
#define DSP402_MAX_TORQUE                     0x6072
#define DSP402_MAX_CURRENT                    0x6073
#define DSP402_TORQUE_DEMAND_VALUE            0x6074
#define DSP402_MOTOR_RATED_CURRENT            0x6075
#define DSP402_MOTOR_RATED_TORQUE             0x6076
#define DSP402_TORQUE_ACTUAL_VALUE            0x6077
#define DSP402_CURRENT_ACTUAL_VALUE           0x6078
#define DSP402_TARGET_POSITION                0x607A
#define DSP402_POSITION_RANGE_LIMIT           0x607B
#define DSP402_HOME_OFFSET                    0x607C
#define DSP402_SOFTWARE_POSITION_LIMIT        0x607D
#define DSP402_POLARITY                       0x607E
#define DSP402_MAX_PROFILE_VELOCITY           0x607F
#define DSP402_PROFILE_VELOCITY               0x6081
#define DSP402_PROFILE_ACCELERATION           0x6083
#define DSP402_PROFILE_DECELERATION           0x6084
#define DSP402_QUICK_STOP_DECELERATION        0x6085
#define DSP402_MOTION_PROFILE_TYPE            0x6086
#define DSP402_TORQUE_SLOPE                   0x6087
#define DSP402_TORQUE_PROFILE_TYPE            0x6088
#define DSP402_POSITION_NOTATION_INDEX        0x6089 
#define DSP402_POSITION_DIMENSION_INDEX       0x608A
#define DSP402_VELOCITY_NOTATION_INDEX        0x608B
#define DSP402_VELOCITY_DIMENSION_INDEX       0x608C
#define DSP402_ACCELERATION_NOTATION_INDEX    0x608D
#define DSP402_ACCELERATION_DIMENSION_INDEX   0x608E
#define DSP402_POSITION_ENCODER_RESOLUTION    0x608F
#define DSP402_VELOCITY_ENCODER_RESOLUTION    0x6090
#define DSP402_HOMING_METHOD                  0x6098
#define DSP402_HOMING_SPEEDS                  0x6099
#define DSP402_HOMING_ACCELERATION            0x609A
#define DSP402_MAX_ACCELERATION               0x60C5
#define DSP402_MAX_DECELERATION               0x60C6
#define DSP402_TORQUE_CONTROL_PARAMETER_SET   0x60F6 // MFE specific
#define DSP402_VELOCITY_CONTROL_PARAMETER_SET 0x60F9 // MFE specific
#define DSP402_POSITION_CONTROL_PARAMETER_SET 0x60FB // MFE specific
#define DSP402_POSITION_DEMAND_VALUE_INTERNAL 0x60FC
#define DSP402_TARGET_VELOCITY                0x60FF
#define DSP402_MOTOR_TYPE                     0x6402
#define DSP402_MOTOR_DATA                     0x6410 // MFE specific
#define DSP402_SUPPORTED_DRIVE_MODES          0x6502

/* defines controlword bit masks */
#define DSP402_CONTROLBIT_SWITCH_ON              0x0001 // Bit 0
#define DSP402_CONTROLBIT_DISABLE_VOLTAGE        0x0002 // Bit 1
#define DSP402_CONTROLBIT_QUICK_STOP             0x0004 // Bit 2
#define DSP402_CONTROLBIT_ENABLE_OPERATION       0x0008 // Bit 3
#define DSP402_CONTROLBIT_NEW_SETPOINT           0x0010 // Bit 4
#define DSP402_CONTROLBIT_HOMING_OPERATION_START 0x0010 // Bit 4
#define DSP402_CONTROLBIT_CHANGE_SET_IMMEDIATELY 0x0020 // Bit 5
#define DSP402_CONTROLBIT_ABSOLUTE_RELATIVE      0x0040 // Bit 6 
#define DSP402_CONTROLBIT_RESET_FAULT            0x0080 // Bit 7
#define DSP402_CONTROLBIT_HALT                   0x0100 // Bit 8

/* defines statusword bit masks */
#define DSP402_STATUSBIT_READY                   0x0001 // Bit 0
#define DSP402_STATUSBIT_SWITCHED_ON             0x0002 // Bit 1
#define DSP402_STATUSBIT_OPERATION_ENABLED       0x0004 // Bit 2
#define DSP402_STATUSBIT_FAULT                   0x0008 // Bit 3
#define DSP402_STATUSBIT_VOLTAGE_DISABLED        0x0010 // Bit 4
#define DSP402_STATUSBIT_QUICK_STOP              0x0020 // Bit 5
#define DSP402_STATUSBIT_SWITCH_ON_DISABLED      0x0040 // Bit 6
#define DSP402_STATUSBIT_WARNING                 0x0080 // Bit 7
#define DSP402_STATUSBIT_REMOTE                  0x0200 // Bit 9
#define DSP402_STATUSBIT_TARGET_REACHED          0x0400 // Bit 10
#define DSP402_STATUSBIT_INTERNAL_LIMIT_ACTIVE   0x0800 // Bit 11
#define DSP402_STATUSBIT_HOMING_ATTAINED         0x1000 // Bit 12
#define DSP402_STATUSBIT_SETPOINT_ACKNOWLEDGE    0x1000 // Bit 12
#define DSP402_STATUSBIT_HOMING_ERROR            0x2000 // Bit 13
#define DSP402_STATUSBIT_FOLLOWING_ERROR         0x2000 // Bit 13

class dsp402_slave : public co_slave {
public:
  dsp402_slave(uint8_t node_id = 1);
  void shutdown();
  void switch_on();
  void disable_voltage();
  void quick_stop();
  void disable_operation();  
  void enable_operation();
  void fault_reset();
  void set_controlbit_switch_on(int value);
  void set_controlbit_disable_voltage(int value);
  void set_controlbit_quick_stop(int value);
  void set_controlbit_enable_operation(int value);
  void set_controlbit_new_setpoint(int value);
  void set_controlbit_homing_operation_start(int value);
  void set_controlbit_change_set_immediately(int value);
  void set_controlbit_absolute_relative(int value);
  void set_controlbit_reset_fault(int value);
  void set_controlbit_halt(int value);
  int get_statusbit_ready();
  int get_statusbit_switched_on();
  int get_statusbit_operation_enabled();
  int get_statusbit_fault();
  int get_statusbit_voltage_disabled();
  int get_statusbit_quick_stop();
  int get_statusbit_switch_on_disabled();
  int get_statusbit_warning();
  int get_statusbit_remote();
  int get_statusbit_target_reached();
  int get_statusbit_internal_limit_active();
  int get_statusbit_homing_attained();
  int get_statusbit_setpoint_acknowledge();
  int get_statusbit_homing_error();
  int get_statusbit_following_error();
};

#endif
