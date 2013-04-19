#ifndef _PID_H_
#define _PID_H_

#include "filter.h"

/******************************************************************************
** pid.o : pid controller
*******************************************************************************/

struct pid {
  double Ts;
  double p_gain;
  double i_gain;
  double d_gain;
  double min_output;
  double max_output;
  double min_setpoint;
  double max_setpoint;
  double setpoint;
  double error;
  double output;
  double p_term;
  double i_term;
  double d_term;
  double d_break_frequency;
  struct filter d_filter;
};

struct pid new_pid(double Ts);
void pid_set_output_limits(struct pid *o, double min_output, double max_output);
void pid_set_setpoint_limits(struct pid *o, double min_setpoint, double max_setpoint);
void pid_set_time_step(struct pid *o, double Ts);
void pid_set_gains(struct pid *o, double p_gain, double i_gain, double d_gain);
void pid_set_p_gain(struct pid *o, double p_gain);
void pid_set_i_gain(struct pid *o, double i_gain);
void pid_set_d_gain(struct pid *o, double d_gain);
void pid_set_d_break_frequency(struct pid *o, double d_break_frequency);
void pid_set_setpoint(struct pid *o, double setpoint);
double pid_get_setpoint(struct pid *o);
double pid_get_process_value(struct pid *o);
double pid_get_derivative(struct pid *o);
void pid_reset(struct pid *o);
double pid_update(struct pid *o, double process_value);

#endif
