/*
 * File:          tiny_math.h
 * Date:          24th May 2011
 * Description:   mathematical functions
 * Author:        fabien.rohrer@cyberbotics.com
 * Modifications: 
 */

#ifndef TINY_MATH_H
#define TINY_MATH_H

#include <webots/types.h>

typedef struct {
  double u;
  double v;
} Vector2;

typedef struct {
  double u;
  double v;
  double w;
} Vector3;

typedef struct {
  Vector3 a;
  Vector3 b;
  Vector3 c;
} Matrix33;

// --- Vector3 functions ---
void vector3_set_values(Vector3 *vect, double u, double v, double w);

// --- Matrix33 functions ---
void  matrix33_set_values(Matrix33 *m, double au, double av, double aw, double bu, double bv, double bw, double cu, double cv, double cw);
void   matrix33_set_identity(Matrix33 *m);
void   matrix33_mult_vector3(Vector3 *res, const Matrix33 *m, const Vector3 *v); // res = m * v

// --- Vector2 functions ---
double vector2_norm(const Vector2 *v); // ||v||
void   vector2_minus(Vector2 *v, const Vector2 *v1, const Vector2 *v2); // v = v1-v2
double vector2_angle(const Vector2 *v1, const Vector2 *v2); // angle between v1 and v2 -> [0, 2Pi]

// --- Other ---
double bound(double v, double a, double b);

#endif
