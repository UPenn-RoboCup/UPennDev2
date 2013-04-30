#include <stdlib.h>
#include <stdio.h>
#include <math.h>

float sqrtA(float x) {
  unsigned int i = *(unsigned int*) &x;
  i += 127 << 23;
  i >>=1;
  return *(float*) &i;
}

int main() {
  int x = 445;
  double r = sqrt(x);
  printf("%f", r);
  return 1;
}
