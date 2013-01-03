/*

  Author: Daniel D. Lee <ddlee@seas.upenn.edu>, 05/10
  	: Stephen McGill 10/10
*/
			
#include "timeScalar.h"

double time_scalar() {
  static struct timeval t;
  gettimeofday(&t, NULL);
  return t.tv_sec + 1E-6*t.tv_usec;
}
