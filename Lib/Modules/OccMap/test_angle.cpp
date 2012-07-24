#include <iostream>
#include <cmath>

int main() {
  double x = -0.0001;
  double y = -5;
  double angle = atan2(x, y);
  std::cout << angle * 180 /3.1415926<< std::endl;
  return 1;
}
