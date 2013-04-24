#include <stdint.h>
#include "co_types.h"
#include "slug_slave.h"

/******************************************************************************
 * slug_slave : CANopen communication interface for motor slug
 * authors : Mike Hopkins, Steve Ressler
 ******************************************************************************/

/* defines motor slug object dictionary entries */
static co_dictionary_entry slug_slave_dictionary[] = {
  {SLUG_NODE_ID, 0x00, CO_UNSIGNED8},
  {SLUG_CAN_BIT_RATE, 0x00, CO_UNSIGNED8},
  {SLUG_OPERATING_MODE, 0x00, CO_UNSIGNED8},
  {SLUG_OPERATING_MODE_DISPLAY, 0x00, CO_UNSIGNED8},
  {SLUG_STATUS_CODE, 0x00, CO_UNSIGNED8},
  {SLUG_ERROR_CODE, 0x00, CO_UNSIGNED8},

  {SLUG_JOINT_FORCE_CONSTANT, 0x00, CO_INTEGER16},
  {SLUG_JOINT_POSITION_CONSTANT, 0x00, CO_INTEGER16},
  {SLUG_JOINT_VELOCITY_CONSTANT, 0x00, CO_INTEGER16},
  {SLUG_JOINT_PID_GAIN_CONSTANT, 0x00, CO_INTEGER16},
  {SLUG_JOINT_FORCE_MAX, 0x01, CO_INTEGER16},
  {SLUG_JOINT_FORCE_MAX, 0x02, CO_INTEGER16},
  {SLUG_JOINT_POSITION_MAX, 0x01, CO_INTEGER16},
  {SLUG_JOINT_POSITION_MAX, 0x02, CO_INTEGER16},
  {SLUG_JOINT_POSITION_MIN, 0x01, CO_INTEGER16},
  {SLUG_JOINT_POSITION_MIN, 0x02, CO_INTEGER16},
  {SLUG_JOINT_VELOCITY_MAX, 0x01, CO_INTEGER16},
  {SLUG_JOINT_VELOCITY_MAX, 0x02, CO_INTEGER16},
  {SLUG_JOINT_FORCE_SETPOINT, 0x01, CO_INTEGER16},
  {SLUG_JOINT_FORCE_SETPOINT, 0x02, CO_INTEGER16},
  {SLUG_JOINT_POSITION_SETPOINT, 0x01, CO_INTEGER16},
  {SLUG_JOINT_POSITION_SETPOINT, 0x02, CO_INTEGER16},
  {SLUG_JOINT_VELOCITY_SETPOINT, 0x01, CO_INTEGER16},
  {SLUG_JOINT_VELOCITY_SETPOINT, 0x02, CO_INTEGER16},
  {SLUG_JOINT_FORCE_ESTIMATE, 0x01, CO_INTEGER16},
  {SLUG_JOINT_FORCE_ESTIMATE, 0x02, CO_INTEGER16},
  {SLUG_JOINT_POSITION_ESTIMATE, 0x01, CO_INTEGER16},
  {SLUG_JOINT_POSITION_ESTIMATE, 0x02, CO_INTEGER16},
  {SLUG_JOINT_VELOCITY_ESTIMATE, 0x01, CO_INTEGER16},
  {SLUG_JOINT_VELOCITY_ESTIMATE, 0x02, CO_INTEGER16},
  {SLUG_JOINT_POSITION_SENSOR_BIAS, 0x01, CO_INTEGER16},
  {SLUG_JOINT_POSITION_SENSOR_BIAS, 0x02, CO_INTEGER16},
  {SLUG_JOINT_POSITION_P_GAIN, 0x01, CO_INTEGER16},
  {SLUG_JOINT_POSITION_P_GAIN, 0x02, CO_INTEGER16},
  {SLUG_JOINT_POSITION_I_GAIN, 0x01, CO_INTEGER16},
  {SLUG_JOINT_POSITION_I_GAIN, 0x02, CO_INTEGER16},
  {SLUG_JOINT_POSITION_D_GAIN, 0x01, CO_INTEGER16},
  {SLUG_JOINT_POSITION_D_GAIN, 0x02, CO_INTEGER16},
  {SLUG_JOINT_POSITION_D_BREAK_FREQUENCY, 0x01, CO_REAL32},
  {SLUG_JOINT_POSITION_D_BREAK_FREQUENCY, 0x02, CO_REAL32},
  {SLUG_JOINT_FORCE_DEMAND, 0x01, CO_INTEGER16},
  {SLUG_JOINT_FORCE_DEMAND, 0x02, CO_INTEGER16},

  {SLUG_MOTOR_FORCE_MAX, 0x01, CO_REAL32},
  {SLUG_MOTOR_FORCE_MAX, 0x02, CO_REAL32},
  {SLUG_MOTOR_CURRENT_MAX, 0x01, CO_REAL32},
  {SLUG_MOTOR_CURRENT_MAX, 0x02, CO_REAL32},
  {SLUG_MOTOR_FORCE_SETPOINT, 0x01, CO_REAL32},
  {SLUG_MOTOR_FORCE_SETPOINT, 0x02, CO_REAL32},
  {SLUG_MOTOR_FORCE_ESTIMATE, 0x01, CO_REAL32},
  {SLUG_MOTOR_FORCE_ESTIMATE, 0x02, CO_REAL32},
  {SLUG_MOTOR_CURRENT_ESTIMATE, 0x01, CO_REAL32},
  {SLUG_MOTOR_CURRENT_ESTIMATE, 0x02, CO_REAL32},
  {SLUG_MOTOR_FORCE_SENSOR_BIAS, 0x01, CO_REAL32},
  {SLUG_MOTOR_FORCE_SENSOR_BIAS, 0x02, CO_REAL32},
  {SLUG_MOTOR_FORCE_FF_CONSTANT, 0x01, CO_REAL32},
  {SLUG_MOTOR_FORCE_FF_CONSTANT, 0x02, CO_REAL32},
  {SLUG_MOTOR_FORCE_P_GAIN, 0x01, CO_REAL32},
  {SLUG_MOTOR_FORCE_P_GAIN, 0x02, CO_REAL32},
  {SLUG_MOTOR_FORCE_I_GAIN, 0x01, CO_REAL32},
  {SLUG_MOTOR_FORCE_I_GAIN, 0x02, CO_REAL32},
  {SLUG_MOTOR_FORCE_D_GAIN, 0x01, CO_REAL32},
  {SLUG_MOTOR_FORCE_D_GAIN, 0x02, CO_REAL32},
  {SLUG_MOTOR_FORCE_D_BREAK_FREQUENCY, 0x01, CO_REAL32},
  {SLUG_MOTOR_FORCE_D_BREAK_FREQUENCY, 0x02, CO_REAL32},
  {SLUG_MOTOR_CURRENT_DEMAND, 0x01, CO_REAL32},
  {SLUG_MOTOR_CURRENT_DEMAND, 0x02, CO_REAL32},
  {CO_SENTINEL}
};

slug_slave::slug_slave(uint8_t node_id) : co_slave(node_id)
{
  /* initialize object dictionary */
  register_dictionary_entries(slug_slave_dictionary);
}

void slug_slave::set_node_id(int node_id)
{
  set_value(SLUG_NODE_ID, 0x00,
    (uint8_t)node_id
  );
}

void slug_slave::set_can_bit_rate(int can_bit_rate)
{
  set_value(SLUG_CAN_BIT_RATE, 0x00,
    (uint8_t)can_bit_rate
  );
}

void slug_slave::set_operating_mode(int operating_mode)
{
  set_value(SLUG_OPERATING_MODE, 0x00,
    (uint8_t)operating_mode
  );
}

void slug_slave::set_operating_mode_display(int operating_mode_display)
{
  set_value(SLUG_OPERATING_MODE_DISPLAY, 0x00,
    (uint8_t)operating_mode_display
  );
}

void slug_slave::set_status_code(int status_code)
{
  set_value(SLUG_STATUS_CODE, 0x00,
    (uint8_t)status_code
  );
}

void slug_slave::set_error_code(int error_code)
{
  set_value(SLUG_ERROR_CODE, 0x00,
    (uint8_t)error_code
  );
}

void slug_slave::set_joint_force_max(double joint_force_max, int id)
{
  set_value(SLUG_JOINT_FORCE_MAX, id,
    (int16_t)(joint_force_max*get_value(SLUG_JOINT_FORCE_CONSTANT, 0x00))
  );
}

void slug_slave::set_joint_position_max(double joint_position_max, int id)
{
  set_value(SLUG_JOINT_POSITION_MAX, id,
    (int16_t)(joint_position_max*get_value(SLUG_JOINT_POSITION_CONSTANT, 0x00))
  );
}

void slug_slave::set_joint_position_min(double joint_position_min, int id)
{
  set_value(SLUG_JOINT_POSITION_MIN, id,
    (int16_t)(joint_position_min*get_value(SLUG_JOINT_POSITION_CONSTANT, 0x00))
  );
}

void slug_slave::set_joint_velocity_max(double joint_velocity_max, int id)
{
  set_value(SLUG_JOINT_VELOCITY_MAX, id,
    (int16_t)(joint_velocity_max*get_value(SLUG_JOINT_VELOCITY_CONSTANT, 0x00))
  );
}

void slug_slave::set_joint_force_setpoint(double joint_force_setpoint, int id)
{
  set_value(SLUG_JOINT_FORCE_SETPOINT, id,
    (int16_t)(joint_force_setpoint*get_value(SLUG_JOINT_FORCE_CONSTANT, 0x00))
  );
}

void slug_slave::set_joint_position_setpoint(double joint_position_setpoint, int id)
{
  set_value(SLUG_JOINT_POSITION_SETPOINT, id,
    (int16_t)(joint_position_setpoint*get_value(SLUG_JOINT_POSITION_CONSTANT, 0x00))
  );
}

void slug_slave::set_joint_velocity_setpoint(double joint_velocity_setpoint, int id)
{
  set_value(SLUG_JOINT_VELOCITY_SETPOINT, id,
    (int16_t)(joint_velocity_setpoint*get_value(SLUG_JOINT_VELOCITY_CONSTANT, 0x00))
  );
}

void slug_slave::set_joint_force_estimate(double joint_force_estimate, int id)
{
  set_value(SLUG_JOINT_FORCE_ESTIMATE, id,
    (int16_t)(joint_force_estimate*get_value(SLUG_JOINT_FORCE_CONSTANT, 0x00))
  );
}

void slug_slave::set_joint_position_estimate(double joint_position_estimate, int id)
{
  set_value(SLUG_JOINT_POSITION_ESTIMATE, id,
    (int16_t)(joint_position_estimate*get_value(SLUG_JOINT_POSITION_CONSTANT, 0x00))
  );
}

void slug_slave::set_joint_velocity_estimate(double joint_velocity_estimate, int id)
{
  set_value(SLUG_JOINT_VELOCITY_ESTIMATE, id,
    (int16_t)(joint_velocity_estimate*get_value(SLUG_JOINT_VELOCITY_CONSTANT, 0x00))
  );
}

void slug_slave::set_joint_position_sensor_bias(double joint_position_sensor_bias, int id)
{
  set_value(SLUG_JOINT_POSITION_SENSOR_BIAS, id,
    (int16_t)(joint_position_sensor_bias*get_value(SLUG_JOINT_POSITION_CONSTANT, 0x00))
  );
}

void slug_slave::set_joint_position_p_gain(double joint_position_p_gain, int id)
{
  set_value(SLUG_JOINT_POSITION_P_GAIN, id,
    (int16_t)(joint_position_p_gain*get_value(SLUG_JOINT_PID_GAIN_CONSTANT, 0x00))
  );
}

void slug_slave::set_joint_position_i_gain(double joint_position_i_gain, int id)
{
  set_value(SLUG_JOINT_POSITION_I_GAIN, id,
    (int16_t)(joint_position_i_gain*get_value(SLUG_JOINT_PID_GAIN_CONSTANT, 0x00))
  );
}

void slug_slave::set_joint_position_d_gain(double joint_position_d_gain, int id)
{
  set_value(SLUG_JOINT_POSITION_D_GAIN, id,
    (int16_t)(joint_position_d_gain*get_value(SLUG_JOINT_PID_GAIN_CONSTANT, 0x00))
  );
}

void slug_slave::set_joint_position_d_break_frequency(double joint_position_d_break_frequency, int id)
{
  set_value(SLUG_JOINT_POSITION_D_BREAK_FREQUENCY, id,
    (float)joint_position_d_break_frequency
  );
}

void slug_slave::set_joint_force_demand(double joint_force_demand, int id)
{
  set_value(SLUG_JOINT_FORCE_DEMAND, id,
    (int16_t)(joint_force_demand*get_value(SLUG_JOINT_FORCE_CONSTANT, 0x00))
  );
}

void slug_slave::set_motor_force_max(double motor_force_max, int id)
{
  set_value(SLUG_MOTOR_FORCE_MAX, id,
    (float)motor_force_max
  );
}

void slug_slave::set_motor_current_max(double motor_current_max, int id)
{
  set_value(SLUG_MOTOR_CURRENT_MAX, id,
    (float)motor_current_max
  );
}

void slug_slave::set_motor_force_setpoint(double motor_force_setpoint, int id)
{
  set_value(SLUG_MOTOR_FORCE_SETPOINT, id,
    (float)motor_force_setpoint
  );
}

void slug_slave::set_motor_force_estimate(double motor_force_estimate, int id)
{
  set_value(SLUG_MOTOR_FORCE_ESTIMATE, id,
    (float)motor_force_estimate
  );
}

void slug_slave::set_motor_current_estimate(double motor_current_estimate, int id)
{
  set_value(SLUG_MOTOR_CURRENT_ESTIMATE, id,
    (float)motor_current_estimate
  );
}

void slug_slave::set_motor_force_sensor_bias(double motor_force_sensor_bias, int id)
{
  set_value(SLUG_MOTOR_FORCE_SENSOR_BIAS, id,
    (float)motor_force_sensor_bias
  );
}

void slug_slave::set_motor_force_ff_constant(double motor_force_ff_constant, int id)
{
  set_value(SLUG_MOTOR_FORCE_FF_CONSTANT, id,
    (float)motor_force_ff_constant
  );

}

void slug_slave::set_motor_force_p_gain(double motor_force_p_gain, int id)
{
  set_value(SLUG_MOTOR_FORCE_P_GAIN, id,
    (float)motor_force_p_gain
  );
}

void slug_slave::set_motor_force_i_gain(double motor_force_i_gain, int id)
{
  set_value(SLUG_MOTOR_FORCE_I_GAIN, id,
    (float)motor_force_i_gain
  );
}

void slug_slave::set_motor_force_d_gain(double motor_force_d_gain, int id)
{
  set_value(SLUG_MOTOR_FORCE_D_GAIN, id,
    (float)motor_force_d_gain
  );
}

void slug_slave::set_motor_force_d_break_frequency(double motor_force_d_break_frequency, int id)
{
  set_value(SLUG_MOTOR_FORCE_D_BREAK_FREQUENCY, id,
    (float)motor_force_d_break_frequency
  );
}

void slug_slave::set_motor_current_demand(double motor_current_demand, int id)
{
  set_value(SLUG_MOTOR_CURRENT_DEMAND, id,
    (float)motor_current_demand
  );
}

int slug_slave::get_node_id(void)
{
  return (int)get_value(SLUG_NODE_ID, 0x00);
}

int slug_slave::get_can_bit_rate(void)
{
  return (int)get_value(SLUG_CAN_BIT_RATE, 0x00);
}

int slug_slave::get_operating_mode(void)
{
  return (int)get_value(SLUG_OPERATING_MODE, 0x00);
}

int slug_slave::get_operating_mode_display(void)
{
  return (int)get_value(SLUG_OPERATING_MODE_DISPLAY, 0x00);
}

int slug_slave::get_status_code(void)
{
  return (int)get_value(SLUG_STATUS_CODE, 0x00);
}

int slug_slave::get_error_code(void)
{
  return (int)get_value(SLUG_ERROR_CODE, 0x00);
}

double slug_slave::get_joint_force_max(int id)
{
  return (double)get_value(SLUG_JOINT_FORCE_MAX, id)
       / (double)get_value(SLUG_JOINT_FORCE_CONSTANT, 0x00);
}

double slug_slave::get_joint_position_max(int id)
{
  return (double)get_value(SLUG_JOINT_POSITION_MAX, id)
       / (double)get_value(SLUG_JOINT_POSITION_CONSTANT, 0x00);
}

double slug_slave::get_joint_position_min(int id)
{
  return (double)get_value(SLUG_JOINT_POSITION_MIN, id)
       / (double)get_value(SLUG_JOINT_POSITION_CONSTANT, 0x00);
}

double slug_slave::get_joint_velocity_max(int id)
{
  return (double)get_value(SLUG_JOINT_VELOCITY_MAX, id)
       / (double)get_value(SLUG_JOINT_VELOCITY_CONSTANT, 0x00);
}

double slug_slave::get_joint_force_setpoint(int id)
{
  return (double)get_value(SLUG_JOINT_FORCE_SETPOINT, id)
       / (double)get_value(SLUG_JOINT_FORCE_CONSTANT, 0x00);
}

double slug_slave::get_joint_position_setpoint(int id)
{
  return (double)get_value(SLUG_JOINT_POSITION_SETPOINT, id)
       / (double)get_value(SLUG_JOINT_POSITION_CONSTANT, 0x00);
}

double slug_slave::get_joint_velocity_setpoint(int id)
{
  return (double)get_value(SLUG_JOINT_VELOCITY_SETPOINT, id)
       / (double)get_value(SLUG_JOINT_VELOCITY_CONSTANT, 0x00);
}

double slug_slave::get_joint_force_estimate(int id)
{
  return (double)get_value(SLUG_JOINT_FORCE_ESTIMATE, id)
       / (double)get_value(SLUG_JOINT_FORCE_CONSTANT, 0x00);
}

double slug_slave::get_joint_position_estimate(int id)
{
  return (double)get_value(SLUG_JOINT_POSITION_ESTIMATE, id)
       / (double)get_value(SLUG_JOINT_POSITION_CONSTANT, 0x00);
}

double slug_slave::get_joint_velocity_estimate(int id)
{
  return (double)get_value(SLUG_JOINT_VELOCITY_ESTIMATE, id)
       / (double)get_value(SLUG_JOINT_VELOCITY_CONSTANT, 0x00);
}

double slug_slave::get_joint_position_sensor_bias(int id)
{
  return (double)get_value(SLUG_JOINT_POSITION_SENSOR_BIAS, id)
       / (double)get_value(SLUG_JOINT_POSITION_CONSTANT, 0x00);
}

double slug_slave::get_joint_position_p_gain(int id)
{
  return (double)get_value(SLUG_JOINT_POSITION_P_GAIN, id)
       / (double)get_value(SLUG_JOINT_PID_GAIN_CONSTANT, 0x00);
}

double slug_slave::get_joint_position_i_gain(int id)
{
  return (double)get_value(SLUG_JOINT_POSITION_I_GAIN, id)
       / (double)get_value(SLUG_JOINT_PID_GAIN_CONSTANT, 0x00);
}

double slug_slave::get_joint_position_d_gain(int id)
{
  return (double)get_value(SLUG_JOINT_POSITION_D_GAIN, id)
       / (double)get_value(SLUG_JOINT_PID_GAIN_CONSTANT, 0x00);
}

double slug_slave::get_joint_position_d_break_frequency(int id)
{
  return (double)get_value(SLUG_JOINT_POSITION_D_BREAK_FREQUENCY, id);
}

double slug_slave::get_joint_force_demand(int id)
{
  return (double)get_value(SLUG_JOINT_FORCE_DEMAND, id)
       / (double)get_value(SLUG_JOINT_FORCE_CONSTANT, 0x00);
}

double slug_slave::get_motor_force_max(int id)
{
  return (double)get_value(SLUG_MOTOR_FORCE_MAX, id);
}

double slug_slave::get_motor_current_max(int id)
{
  return (double)get_value(SLUG_MOTOR_CURRENT_MAX, id);
}

double slug_slave::get_motor_force_setpoint(int id)
{
  return (double)get_value(SLUG_MOTOR_FORCE_SETPOINT, id);
}

double slug_slave::get_motor_force_estimate(int id)
{
  return (double)get_value(SLUG_MOTOR_FORCE_ESTIMATE, id);
}

double slug_slave::get_motor_current_estimate(int id)
{
  return (double)get_value(SLUG_MOTOR_CURRENT_ESTIMATE, id);
}

double slug_slave::get_motor_force_sensor_bias(int id)
{
  return (double)get_value(SLUG_MOTOR_FORCE_SENSOR_BIAS, id);
}

double slug_slave::get_motor_force_ff_constant(int id)
{
  return (double)get_value(SLUG_MOTOR_FORCE_FF_CONSTANT, id);
}

double slug_slave::get_motor_force_p_gain(int id)
{
  return (double)get_value(SLUG_MOTOR_FORCE_P_GAIN, id);
}

double slug_slave::get_motor_force_i_gain(int id)
{
  return (double)get_value(SLUG_MOTOR_FORCE_I_GAIN, id);
}

double slug_slave::get_motor_force_d_gain(int id)
{
  return (double)get_value(SLUG_MOTOR_FORCE_D_GAIN, id);
}

double slug_slave::get_motor_force_d_break_frequency(int id)
{
  return (double)get_value(SLUG_MOTOR_FORCE_D_BREAK_FREQUENCY, id);
}

double slug_slave::get_motor_current_demand(int id)
{
  return (double)get_value(SLUG_MOTOR_CURRENT_DEMAND, id);
}
