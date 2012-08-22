#ifndef LUT_H_
#define LUT_H_

#include <math.h>

/* Classes for storing and interpolating look-up tables */

class Lut1d {
public:
  double interpolate(double x) const;
  Lut1d(const double imin, const double imax, const int idim, const double *idata);
private:
  const double min;     /* minimum x value */
  const double max;     /* maximum x value */
  const int dim;        /* dimension of the lookup table */
  const double *data;   /* pointer to first element in the data array */
};

class Lut2d {
public:
  double interpolate(double x, double y) const;
  Lut2d(const double imin[2], const double imax[2], const int idim[2], const double *idata);
private:
  const double *min;    /* minimum x and y values */
  const double *max;    /* maximum x and y values */
  const int *dim;       /* dimensions of the lookup table */
  const double *data;   /* pointer to first element in the 2D data array */
};

class Lut3d {
public:
  double interpolate(double x, double y, double z) const;
  Lut3d(const double imin[3], const double imax[3], const int idim[3], const double *idata);
private:
  const double *min;    /* minimum x, y and z values */
  const double *max;    /* maximum x, y and z values */
  const int *dim;       /* dimensions of the lookup table */
  const double *data;   /* pointer to first element in the 3D data array */
};

#endif
