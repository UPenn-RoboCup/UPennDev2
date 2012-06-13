#include <stdlib.h>
#include <stdio.h>
#include <math.h>

float sqrtA(float x) {
  unsigned int i = *(unsigned int*) &x;
  i += 127 << 23;
  i >>=1;
  return *(float*) &i;
}

void main() {
  float x = 4.435;
  float r = sqrtA(x);
  printf("%f", r);
}
