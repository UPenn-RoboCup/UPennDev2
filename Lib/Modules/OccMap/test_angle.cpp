#include <iostream>
#include <cmath>

int main() {
  double a = -1.02088;
  double x = cos(a);
  double y = sin(a);
  double angle = atan2(y, x);
  std::cout << angle << std::endl;
  return 1;
}
