#include "HoughTransform.hh"
#include "Timer.hh"
#include <string.h>

using namespace Upenn;

int CalculateHoughTransform(double * xs, double * ys, int n_points,
        vector<double> & thetas, double rho_center, double & rho_range,
				double rho_res, int * h_transform)
{

	int n_thetas=thetas.size();
	int rho_range_size = (int)(rho_range/rho_res);
	rho_range = rho_range_size*rho_res;
	int n_rhos = 2*rho_range_size+1;
	double rho_min = rho_center - rho_range;
  double inv_rho_res = 1.0 / rho_res;
  Timer timer0;
  timer0.Tic();

	//initialize the h_transform array
	//for (int i=0; i < n_thetas*n_rhos; i++) h_transform[i]=0;
  memset(h_transform,0,n_thetas*n_rhos*sizeof(int));


  //y=tan(th)*x + rho

	//compute the Hough Transform
  double * pthetas = &(thetas[0]);
  int theta_i_times_n_rhos=0;
	for (int theta_i = 0; theta_i < n_thetas; theta_i++) {
    double costh = cos(*pthetas);
    double sinth = sin(*pthetas++);

    double * xxs = xs;
    double * yys = ys;

    int * ph = &(h_transform[theta_i_times_n_rhos]);

    for (int p = 0; p < n_points; p++) {
      double rho = (*xxs++)*sinth -(*yys++)*costh;
      
      //int rho_i = (rho-rho_min)*inv_rho_res;
      //if ((rho_i >= 0) && (rho_i < n_rhos))
      unsigned int rho_i = (rho-rho_min)*inv_rho_res;
      if (rho_i < n_rhos)
				ph[rho_i] ++;
    }

    theta_i_times_n_rhos+=n_rhos;
  }
  //timer0.Toc("Hough Transform");
	return 0;
}

//generate explicit values that cover the given range with specific resolution, centered about given value
int GenerateRange(double center, double & range, double res,
									vector<double> & data, int & num_data){
	int range_size = (int)(range/res);
	num_data = 2*range_size+1;
	
	//correct the range to make it a multiple of step size
	//and update it for outside usage (that's why using reference..)
	range = range_size*res;
	double start = center - range;
	data.resize(num_data);
	data[0] = start;
	for (int i=1; i<num_data; i++){
		data[i]=data[i-1]+res;
		//std::cout<<data[i]<<" ";
	}
	//std::cout<<std::endl;
	
	return 0;
}

