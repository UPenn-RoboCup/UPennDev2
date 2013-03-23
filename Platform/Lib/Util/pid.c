#include "pid.h"
#include <float.h>

/******************************************************************************
** pid.o : pid controller (michael-hopkins@outlook.com)
*******************************************************************************/

#define MAX(A, B) (((A) > (B)) ? (A) : (B))
#define MIN(A, B) (((A) < (B)) ? (A) : (B))

struct pid new_pid(double Ts)
{
  struct pid o;
  o.Ts = Ts;      // sample time
  o.p_gain = 0;
  o.i_gain = 0;
  o.d_gain = 0;
  o.min_output =-DBL_MAX;
  o.max_output = DBL_MAX;
  o.min_setpoint =-DBL_MAX;
  o.max_setpoint = DBL_MAX;
  o.setpoint = 0;
  o.error = 0;
  o.output = 0;
  o.p_term = 0;
  o.i_term = 0;
  o.d_term = 0;
  o.d_break_frequency = 10;
  o.d_filter = new_differentiator(o.Ts, o.d_break_frequency);
  return o;
}

void pid_set_output_limits(struct pid *o, double min_output, double max_output)
{
  o->min_output = min_output;
  o->max_output = max_output;
}

void pid_set_setpoint_limits(struct pid *o, double min_setpoint, double max_setpoint)
{
  o->min_setpoint = min_setpoint;
  o->max_setpoint = max_setpoint;
}

void pid_set_time_step(struct pid *o, double Ts)
{
  o->Ts = Ts;
  o->d_break_frequency = MIN(o->d_break_frequency, 1/(2*o->Ts));
  o->d_filter = new_differentiator(o->Ts, o->d_break_frequency);
}

void pid_set_gains(struct pid *o, double p_gain, double i_gain, double d_gain)
{
  if (p_gain != o->p_gain)
  {
    o->p_gain = p_gain;
    o->p_term = 0;
  }
  if (i_gain != o->i_gain)
  {
    o->i_gain = i_gain;
    o->i_term = 0;
  }
  if (d_gain != o->d_gain)
  {
    o->d_gain = d_gain;
    o->d_term = 0;
  }
}

void pid_set_p_gain(struct pid *o, double p_gain)
{
  if (p_gain != o->p_gain)
  {
    o->p_gain = p_gain;
    o->p_term = 0;
  }
}

void pid_set_i_gain(struct pid *o, double i_gain)
{
  if (i_gain != o->i_gain)
  {
    o->i_gain = i_gain;
    o->i_term = 0;
  }
}

void pid_set_d_gain(struct pid *o, double d_gain)
{
  if (d_gain != o->d_gain)
  {
    o->d_gain = d_gain;
    o->d_term = 0;
  }
}

void pid_set_d_break_frequency(struct pid *o, double d_break_frequency)
{
  o->d_break_frequency = d_break_frequency;
  o->d_break_frequency = MIN(o->d_break_frequency, 1/(2*o->Ts));
  o->d_filter = new_differentiator(o->Ts, o->d_break_frequency);
}

void pid_set_setpoint(struct pid *o, double setpoint)
{
  o->setpoint = MAX(MIN(setpoint, o->max_setpoint), o->min_setpoint);
}

double pid_get_setpoint(struct pid *o)
{
  return o->setpoint;
}

double pid_get_process_value(struct pid *o)
{
  return o->d_filter.input[0];
}

double pid_get_derivative(struct pid *o)
{
  return o->d_filter.output[0];
}

void pid_reset(struct pid *o)
{
  o->p_term = 0;
  o->i_term = 0;
  o->d_term = 0;
  o->error = 0;
  filter_reset(&(o->d_filter));
}

double pid_update(struct pid *o, double process_value)
{
  double e = o->setpoint - process_value;
  double e_mean = (e + o->error)/2;
  double d = filter_update(&(o->d_filter), process_value);

  // update pid terms
  double p_term = o->p_gain*e;
  double i_term = o->i_gain*e_mean*o->Ts + o->i_term;
  double d_term = o->d_gain*-d;

  // calculate output
  double output = p_term + i_term + d_term;
  output = MAX(MIN(output, o->max_output), o->min_output);

  // update control variables
  o->error = e;
  o->output = output;
  o->p_term = p_term;
  o->d_term = d_term;
  if ((output < o->max_output) && (output > o->min_output))
    o->i_term = i_term;

  return output;
}
