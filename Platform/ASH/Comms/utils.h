#ifndef _UTILS_H_
#define _UTILS_H_

#include <time.h>

static inline double get_time() 
{ 
  // return monotonic system time in seconds
  timespec ts;
#ifdef CLOCK_MONOTONIC_RAW
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
#else
  clock_gettime(CLOCK_MONOTONIC, &ts);
#endif
  return ts.tv_sec + ts.tv_nsec*1e-9;
}

static inline double max(double a, double b)
{
  return (a > b) ? a : b;
}

static inline double min(double a, double b)
{
  return (a < b) ? a : b;
}

#endif
