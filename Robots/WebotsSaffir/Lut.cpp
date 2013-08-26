#include "Lut.h"

Lut1d::Lut1d(const double imin, const double imax, const int idim, const double *idata)
:min(imin), max(imax), dim(idim), data(idata)
{
}

Lut2d::Lut2d(const double imin[2], const double imax[2], const int idim[2], const double *idata)
:min(imin), max(imax), dim(idim), data(idata)
{
}

Lut3d::Lut3d(const double imin[3], const double imax[3], const int idim[3], const double *idata)
:min(imin), max(imax), dim(idim), data(idata)
{
}

double Lut1d::interpolate(double x) const
{
  int index;
  double alpha, value;

  /* calculate nearest neighbors and weights */
  double norm = (dim - 1)*(x - min)/(max - min);
  if (norm <= 0)
    norm = 0;
  if (norm >= (dim - 1))
    norm = (dim - 1) - 1e-9;
  index = floor(norm);
  alpha = 1 - (norm - floor(norm));

  /* linear interpolate */
  value = alpha*data[index] + (1-alpha)*data[index + 1];
  return value; 
}

double Lut2d::interpolate(double x, double y) const
{
  int index[2][2];
  double alpha[2][2], value;
  double (*ptr)[dim[1]] = (double (*)[dim[1]])data;
  double p[2] = {x, y};

  /* calculate nearest neighbors and weights */
  for (int i = 0; i < 2; i++) {
    double norm = (dim[i] - 1)*(p[i] - min[i])/(max[i] - min[i]);
    if (norm <= 0)
      norm = 0;
    if (norm >= (dim[i] - 1))
      norm = (dim[i] - 1) - 1e-9;
    index[i][0] = floor(norm);
    index[i][1] = floor(norm) + 1;
    alpha[i][0] = 1 - (norm - floor(norm));
    alpha[i][1] = norm - floor(norm);
  }
  
  /* linear interpolate */
  value = 0;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++)
      value += alpha[0][i]*alpha[1][j]*ptr[index[0][i]][index[1][j]];
  }
  return value; 
}

double Lut3d::interpolate(double x, double y, double z) const
{
  int index[3][2];
  double alpha[3][2], value;
  double (*ptr)[dim[1]][dim[2]] = (double (*)[dim[1]][dim[2]])data;
  double p[3] = {x, y, z};

  /* calculate nearest neighbors and weights */
  for (int i = 0; i < 3; i++) {
    double norm = (dim[i] - 1)*(p[i] - min[i])/(max[i] - min[i]);
    if (norm <= 0)
      norm = 0;
    if (norm >= (dim[i] - 1))
      norm = (dim[i] - 1) - 1e-9;
    index[i][0] = floor(norm);
    index[i][1] = floor(norm) + 1;
    alpha[i][0] = 1 - (norm - floor(norm));
    alpha[i][1] = norm - floor(norm);
  }
  
  /* linear interpolate */
  value = 0;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      for (int k = 0; k < 2; k++)
        value += alpha[0][i]*alpha[1][j]*alpha[2][k]*ptr[index[0][i]][index[1][j]][index[2][k]];
    }
  }
  return value; 
}
