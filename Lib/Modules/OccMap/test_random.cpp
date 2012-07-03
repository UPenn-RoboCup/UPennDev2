#include <iostream>
#include <ctime>
#include <cstdlib>

int main() {
  srand(time(0));
  int max_range = 100;
  int r = (rand() % max_range) + 1;
  std::cout << r << std::endl;
  return 1;
}
