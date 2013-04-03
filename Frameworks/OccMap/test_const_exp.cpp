#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>

using namespace std;

int main() {
  int rem = 50 % 50;
  int quo = (50 - rem ) / 50;
  cout << rem << ' ' << quo << endl;
  double var_x = 0.015;
  double var_y = 0.035;
  double gau_theta = M_PI/4;
  double gau_a = 0.5 * pow(cos(gau_theta) / var_x, 2) + 
                0.5 * pow(sin(gau_theta) / var_y, 2);
  double gau_b = -0.25 * sin(2 * gau_theta) / pow(var_x, 2) +
                     0.25 * sin(2 * gau_theta) / pow(var_y, 2);
  double gau_c = 0.5 * pow(sin(gau_theta) / var_x, 2) +
                    0.5 * pow(cos(gau_theta) / var_y, 2);
  double x_robot = 0;
  double y_robot = 0;
  double ob_x, ob_y, grid_p;
  cout << '{';
  for (int i = -3; i <= 3; i++) {
    cout << '{';
    for (int j = -3; j <= 3; j++) {
      ob_x = i * 0.02;
      ob_y = j * 0.02;
      grid_p = exp(- (gau_a * (x_robot - ob_x) * (x_robot - ob_x) + 
                    2 * gau_b * (x_robot - ob_x) * (y_robot - ob_y) + 
                        gau_c * (y_robot - ob_y) * (y_robot - ob_y)));
      cout << setw(12) << grid_p << ',';
    }
    if (i == 3) cout << "}}" << endl;
    else cout << "}," << endl;
  }
  vector<double> a;
  a.push_back(34.5342);
  vector<double>::iterator i = a.begin();
  std::cout << (int)i << std::endl;
  return 1;
}
