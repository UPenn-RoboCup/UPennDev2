#include <vector>
#include <math.h>
#include <iostream>

using namespace std;

int CalculateHoughTransform(double * xs, double * ys, int n_points, 
        vector<double> & thetas, double rho_center, double & rho_range,
				double rho_res, int * h_transform);

int GenerateRange(double center, double & range, double step,
									vector<double> & data, int & num_data);
