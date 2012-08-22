/*
 * dsp402_slave : base class for dsp402 compliant slave devices 
 * author : Mike Hopkins
 */

#include "co_types.h"
#include "co_slave.h"
#include "dsp402_slave.h"

/* defines standard dsp402 object dictionary entries */
static co_dictionary_entry dsp402_slave_dictionary[] = {
  {DSP402_ABORT_CONNECTION_OPTION_CODE, 0x00, CO_INTEGER16},
  {DSP402_ERROR_CODE, 0x00, CO_UNSIGNED16},
  {DSP402_CONTROLWORD, 0x00, CO_UNSIGNED16},
  {DSP402_STATUSWORD, 0x00, CO_UNSIGNED16},
  {DSP402_QUICK_STOP_OPTION_CODE, 0x00, CO_INTEGER16},
  {DSP402_SHUTDOWN_OPTION_CODE, 0x00, CO_INTEGER16},
  {DSP402_DISABLE_OPERATION_OPTION_CODE, 0x00, CO_INTEGER16},
  {DSP402_STOP_OPTION_CODE, 0x00, CO_INTEGER16},
  {DSP402_FAULT_REACTION_OPTION_CODE, 0x00, CO_INTEGER16},
  {DSP402_MODES_OF_OPERATION, 0x00, CO_INTEGER8},
  {DSP402_MODES_OF_OPERATION_DISPLAY, 0x00, CO_INTEGER8},
  {DSP402_POSITION_DEMAND_VALUE, 0x00, CO_INTEGER32},
  {DSP402_POSITION_ACTUAL_VALUE_INTERNAL, 0x00, CO_INTEGER32},
  {DSP402_POSITION_ACTUAL_VALUE, 0x00, CO_INTEGER32},
  {DSP402_FOLLOWING_ERROR_WINDOW, 0x00, CO_UNSIGNED32},
  {DSP402_FOLLOWING_ERROR_TIME_OUT, 0x00, CO_UNSIGNED16},
  {DSP402_POSITION_WINDOW, 0x00, CO_UNSIGNED32},
  {DSP402_POSITION_WINDOW_TIME, 0x00, CO_UNSIGNED16},
  {DSP402_VELOCITY_SENSOR_ACTUAL_VALUE, 0x00, CO_INTEGER32},
  {DSP402_SENSOR_SELECTION_CODE, 0x00, CO_INTEGER16},
  {DSP402_VELOCITY_DEMAND_VALUE, 0x00, CO_INTEGER32},
  {DSP402_VELOCITY_ACTUAL_VALUE, 0x00, CO_INTEGER32},
  {DSP402_VELOCITY_WINDOW, 0x00, CO_UNSIGNED16},
  {DSP402_VELOCITY_WINDOW_TIME, 0x00, CO_UNSIGNED16},
  {DSP402_TARGET_TORQUE, 0x00, CO_INTEGER16},
  {DSP402_MAX_TORQUE, 0x00, CO_UNSIGNED16},
  {DSP402_MAX_CURRENT, 0x00, CO_UNSIGNED16},
  {DSP402_TORQUE_DEMAND_VALUE, 0x00, CO_INTEGER16},
  {DSP402_MOTOR_RATED_CURRENT, 0x00, CO_UNSIGNED32},
  {DSP402_MOTOR_RATED_TORQUE, 0x00, CO_UNSIGNED32},
  {DSP402_TORQUE_ACTUAL_VALUE, 0x00, CO_INTEGER16},
  {DSP402_CURRENT_ACTUAL_VALUE, 0x00, CO_INTEGER16},
  {DSP402_TARGET_POSITION, 0x00, CO_INTEGER32},
  {DSP402_POSITION_RANGE_LIMIT, 0x01, CO_INTEGER32},
  {DSP402_POSITION_RANGE_LIMIT, 0x02, CO_INTEGER32},
  {DSP402_HOME_OFFSET, 0x00, CO_INTEGER32},
  {DSP402_SOFTWARE_POSITION_LIMIT, 0x01, CO_INTEGER32},
  {DSP402_SOFTWARE_POSITION_LIMIT, 0x02, CO_INTEGER32},
  {DSP402_POLARITY, 0x00, CO_UNSIGNED8},
  {DSP402_MAX_PROFILE_VELOCITY, 0x00, CO_UNSIGNED32},
  {DSP402_PROFILE_VELOCITY, 0x00, CO_UNSIGNED32},
  {DSP402_PROFILE_ACCELERATION, 0x00, CO_UNSIGNED32},
  {DSP402_PROFILE_DECELERATION, 0x00, CO_UNSIGNED32},
  {DSP402_QUICK_STOP_DECELERATION, 0x00, CO_UNSIGNED32},
  {DSP402_MOTION_PROFILE_TYPE, 0x00, CO_INTEGER16},
  {DSP402_TORQUE_SLOPE, 0x00, CO_UNSIGNED32},
  {DSP402_TORQUE_PROFILE_TYPE, 0x00, CO_INTEGER16},
  {DSP402_POSITION_NOTATION_INDEX, 0x00, CO_INTEGER8},
  {DSP402_POSITION_DIMENSION_INDEX, 0x00, CO_UNSIGNED8},
  {DSP402_VELOCITY_NOTATION_INDEX, 0x00, CO_INTEGER8},
  {DSP402_VELOCITY_DIMENSION_INDEX, 0x00, CO_UNSIGNED8},
  {DSP402_ACCELERATION_NOTATION_INDEX, 0x00, CO_INTEGER8},
  {DSP402_ACCELERATION_DIMENSION_INDEX, 0x00, CO_UNSIGNED8},
  {DSP402_POSITION_ENCODER_RESOLUTION, 0x01, CO_UNSIGNED32},
  {DSP402_POSITION_ENCODER_RESOLUTION, 0x02, CO_UNSIGNED32},
  {DSP402_VELOCITY_ENCODER_RESOLUTION, 0x01, CO_UNSIGNED32},
  {DSP402_VELOCITY_ENCODER_RESOLUTION, 0x02, CO_UNSIGNED32},
  {DSP402_HOMING_METHOD, 0x00, CO_INTEGER8},
  {DSP402_HOMING_SPEEDS, 0x01, CO_UNSIGNED32},
  {DSP402_HOMING_SPEEDS, 0x02, CO_UNSIGNED32},
  {DSP402_HOMING_ACCELERATION, 0x00, CO_UNSIGNED32},
  {DSP402_MAX_ACCELERATION, 0x00, CO_UNSIGNED32},
  {DSP402_MAX_DECELERATION, 0x00, CO_UNSIGNED32},
  {DSP402_POSITION_DEMAND_VALUE_INTERNAL, 0x00, CO_INTEGER32},
  {DSP402_TARGET_VELOCITY, 0x00, CO_INTEGER32},
  {DSP402_MOTOR_TYPE, 0x00, CO_UNSIGNED16},
  {DSP402_SUPPORTED_DRIVE_MODES, 0x00, CO_UNSIGNED32},
  {CO_SENTINEL}
};

dsp402_slave::dsp402_slave(uint8_t node_id) : co_slave(node_id)
{
  /* initialize object dictionary */
  register_dictionary_entries(dsp402_slave_dictionary);
  set_value(DSP402_CONTROLWORD, 0x00, 0);
  set_value(DSP402_STATUSWORD, 0x00, 0);
}

void dsp402_slave::shutdown()
{
  /* set controlword to shutdown command */
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  *controlword |= 0x06;
  *controlword &= ~0x81;
}

void dsp402_slave::switch_on()
{
  /* set controlword to switch on command */
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  *controlword |= 0x07;
  *controlword &= ~0x80;
}

void dsp402_slave::disable_voltage()
{
  /* set controlword to disable voltage command */
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  *controlword &= ~0x82;
}

void dsp402_slave::quick_stop()
{
  /* set controlword to quick stop command */
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  *controlword |= 0x02;
  *controlword &= ~0x84;
}

void dsp402_slave::disable_operation()
{
  /* set controlword to disable operation command */
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  *controlword |= 0x07;
  *controlword &= ~0x88;
}

void dsp402_slave::enable_operation()
{
  /* set controlword to enable operation command */
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  *controlword |= 0x0F;
  *controlword &= ~0x80;
}

void dsp402_slave::fault_reset()
{
  /* set controlword to fault reset stop command */
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  *controlword |= 0x80;
}

void dsp402_slave::set_controlbit_switch_on(int value)
{
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  if (value)
    *controlword |= DSP402_CONTROLBIT_SWITCH_ON;
  else
    *controlword &= ~DSP402_CONTROLBIT_SWITCH_ON;
}

void dsp402_slave::set_controlbit_disable_voltage(int value)
{
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  if (value)
    *controlword |= DSP402_CONTROLBIT_DISABLE_VOLTAGE;
  else
    *controlword &= ~DSP402_CONTROLBIT_DISABLE_VOLTAGE;
}

void dsp402_slave::set_controlbit_quick_stop(int value)
{
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  if (value)
    *controlword |= DSP402_CONTROLBIT_QUICK_STOP;
  else
    *controlword &= ~DSP402_CONTROLBIT_QUICK_STOP;
}

void dsp402_slave::set_controlbit_enable_operation(int value)
{
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  if (value)
    *controlword |= DSP402_CONTROLBIT_ENABLE_OPERATION;
  else
    *controlword &= ~DSP402_CONTROLBIT_ENABLE_OPERATION;
}

void dsp402_slave::set_controlbit_new_setpoint(int value)
{
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  if (value)
    *controlword |= DSP402_CONTROLBIT_NEW_SETPOINT;
  else
    *controlword &= ~DSP402_CONTROLBIT_NEW_SETPOINT;
}

void dsp402_slave::set_controlbit_homing_operation_start(int value)
{
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  if (value)
    *controlword |= DSP402_CONTROLBIT_HOMING_OPERATION_START;
  else
    *controlword &= ~DSP402_CONTROLBIT_HOMING_OPERATION_START;
}

void dsp402_slave::set_controlbit_change_set_immediately(int value)
{
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  if (value)
    *controlword |= DSP402_CONTROLBIT_CHANGE_SET_IMMEDIATELY;
  else
    *controlword &= ~DSP402_CONTROLBIT_CHANGE_SET_IMMEDIATELY;
}

void dsp402_slave::set_controlbit_absolute_relative(int value)
{
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  if (value)
    *controlword |= DSP402_CONTROLBIT_ABSOLUTE_RELATIVE;
  else
    *controlword &= ~DSP402_CONTROLBIT_ABSOLUTE_RELATIVE;
}

void dsp402_slave::set_controlbit_reset_fault(int value)
{
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  if (value)
    *controlword |= DSP402_CONTROLBIT_RESET_FAULT;
  else
    *controlword &= ~DSP402_CONTROLBIT_RESET_FAULT;
}

void dsp402_slave::set_controlbit_halt(int value)
{
  uint16_t *controlword = (uint16_t *)get_data_pointer(DSP402_CONTROLWORD, 0x00);
  if (value)
    *controlword |= DSP402_CONTROLBIT_HALT;
  else
    *controlword &= ~DSP402_CONTROLBIT_HALT;
}

int dsp402_slave::get_statusbit_ready()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_READY ? 1 : 0;
}

int dsp402_slave::get_statusbit_switched_on()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_SWITCHED_ON ? 1 : 0;
}

int dsp402_slave::get_statusbit_operation_enabled()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_OPERATION_ENABLED ? 1 : 0;
}

int dsp402_slave::get_statusbit_fault()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_FAULT ? 1 : 0;
}

int dsp402_slave::get_statusbit_voltage_disabled()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_VOLTAGE_DISABLED ? 1 : 0;
}

int dsp402_slave::get_statusbit_quick_stop()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_QUICK_STOP ? 1 : 0;
}

int dsp402_slave::get_statusbit_switch_on_disabled()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_SWITCH_ON_DISABLED ? 1 : 0;
}

int dsp402_slave::get_statusbit_warning()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_WARNING ? 1 : 0;
}

int dsp402_slave::get_statusbit_remote()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_REMOTE ? 1 : 0;
}

int dsp402_slave::get_statusbit_target_reached()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_TARGET_REACHED ? 1 : 0;
}

int dsp402_slave::get_statusbit_internal_limit_active()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_INTERNAL_LIMIT_ACTIVE ? 1 : 0;
}

int dsp402_slave::get_statusbit_homing_attained()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_HOMING_ATTAINED ? 1 : 0;
}

int dsp402_slave::get_statusbit_setpoint_acknowledge()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_SETPOINT_ACKNOWLEDGE ? 1 : 0;
}

int dsp402_slave::get_statusbit_homing_error()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_HOMING_ERROR ? 1 : 0;
}

int dsp402_slave::get_statusbit_following_error()
{
  uint16_t *statusword = (uint16_t *)get_data_pointer(DSP402_STATUSWORD, 0x00);
  return *statusword & DSP402_STATUSBIT_FOLLOWING_ERROR ? 1 : 0;
}
