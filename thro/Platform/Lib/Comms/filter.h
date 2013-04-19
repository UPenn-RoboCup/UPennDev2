#ifndef _FILTER_H_
#define _FILTER_H_

/******************************************************************************
** filter.o : FIR/IIR digital filter
*******************************************************************************/

#define MAX_FILTER_SIZE 64

struct filter {
  double b[MAX_FILTER_SIZE];
  double a[MAX_FILTER_SIZE];
  double input[MAX_FILTER_SIZE];
  double output[MAX_FILTER_SIZE];
  double min_output;
  double max_output;
  int size_b;
  int size_a;
  int initialized;
};

struct filter new_filter(double b[], double a[], int size_b, int size_a);
struct filter new_integrator(double Ts);
struct filter new_differentiator(double Ts, double f);
struct filter new_low_pass(double Ts, double f);
struct filter new_second_order_differentiator(double Ts, double f, double Q);
struct filter new_second_order_low_pass(double Ts, double f, double Q);
void filter_set_output_limits(struct filter *o,
     double min_output, double max_output);
void filter_reset(struct filter *o);
double filter_update(struct filter *o, double input);

#endif
