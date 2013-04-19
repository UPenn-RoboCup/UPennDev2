#include "filter.h"
#include <string.h>
#include <float.h>

/******************************************************************************
** filter.o : FIR/IIR digital filter (michael-hopkins@outlook.com)
*******************************************************************************/

#define PI 3.14159265
#define MAX(A, B) (((A) > (B)) ? (A) : (B))
#define MIN(A, B) (((A) < (B)) ? (A) : (B))

struct filter new_filter(double b[], double a[], int size_b, int size_a)
{
  // b : numerator coefficients of z-domain transfer function
  // a : denominator coefficients of z-domain transfer function
  int i;
  struct filter o;
  o.initialized = 0;
  o.size_b = MIN(size_b, MAX_FILTER_SIZE);
  o.size_a = MIN(size_a, MAX_FILTER_SIZE);
  memset(o.input, 0, MAX_FILTER_SIZE);
  memset(o.output, 0, MAX_FILTER_SIZE);
  o.min_output =-DBL_MAX;
  o.max_output = DBL_MAX;
  for (i = 0; i < size_b; i++)
    o.b[i] = b[i]/a[0];
  for (i = 0; i < size_a; i++)
    o.a[i] = a[i]/a[0];
  return o;
}

struct filter new_integrator(double Ts)
{
  double b[2] = {Ts/2, Ts/2};
  double a[2] = {1, -1};
  return new_filter(b, a, 2, 2);
}

struct filter new_differentiator(double Ts, double f)
{
  double w = 2*PI*f;
  double b[2] = {2*w, -2*w};
  double a[2] = {w*Ts + 2, w*Ts - 2};
  return new_filter(b, a, 2, 2);
}

struct filter new_low_pass(double Ts, double f)
{
  double w = 2*PI*f;
  double b[2] = {w*Ts, w*Ts};
  double a[2] = {w*Ts + 2, w*Ts - 2};
  return new_filter(b, a, 2, 2);
}

struct filter new_second_order_differentiator(double Ts, double f, double Q)
{
  double w = 2*PI*f;
  double b[3];
  double a[3];
  b[0] = 2*w*w*Ts;
  b[1] = 0;
  b[2] = -2*w*w*Ts;
  a[0] = w*w*Ts*Ts + 2*w*Ts/Q + 4;
  a[1] = 2*w*w*Ts*Ts - 8;
  a[2] = w*w*Ts*Ts - 2*w*Ts/Q + 4;
  return new_filter(b, a, 3, 3);
}

struct filter new_second_order_low_pass(double Ts, double f, double Q)
{
  double w = 2*PI*f;
  double b[3];
  double a[3];
  b[0] = w*w*Ts*Ts;
  b[1] = 2*w*w*Ts*Ts;
  b[2] = w*w*Ts*Ts;
  a[0] = w*w*Ts*Ts + 2*w*Ts/Q + 4;
  a[1] = 2*w*w*Ts*Ts - 8;
  a[2] = w*w*Ts*Ts - 2*w*Ts/Q + 4;
  return new_filter(b, a, 3, 3);
}

void filter_set_output_limits(struct filter *o, double min_output, double max_output)
{
  o->min_output = min_output;
  o->max_output = max_output;
}

void filter_reset(struct filter *o)
{
  o->initialized = 0;
}

double filter_update(struct filter *o, double input)
{
  int i;

  // initialize input / output history
  if (!o->initialized) 
  {
    memset(o->input, input, o->size_b);
    memset(o->output, 0, o->size_a);
    o->initialized = 1;
  }

  // update input / output history
  for (i = o->size_b-1; i > 0; i--)
    o->input[i] = o->input[i-1];
  o->input[0] = input;
  for (i = o->size_a-1; i > 0; i--)
    o->output[i] = o->output[i-1];
  o->output[0] = 0;

  // return linear filter output
  for (i = 0; i < o->size_b; i++)
    o->output[0] += o->b[i]*o->input[i];
  for (i = 1; i < o->size_a; i++)
    o->output[0] -= o->a[i]*o->output[i];
  o->output[0] = MIN(o->output[0], o->max_output);
  o->output[0] = MAX(o->output[0], o->min_output);
  return o->output[0];
}
