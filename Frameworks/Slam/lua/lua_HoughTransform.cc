#ifdef __cplusplus
extern "C" {
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#ifdef __cplusplus
}
#endif

#include "HoughTransform.hh"
#include <iostream>
#include "Timer.hh"
#include <string.h>

// Arguments
enum {XS,YS,A_CENTER,A_RANGE,A_RES,R_CENTER,R_RANGE,R_RES};

using namespace std;

vector<double> angles;
vector<double> rhos;
int * h_transform = NULL;
int nh = 0;

int lua_HoughTransform(lua_State *L)
//void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  Upenn::Timer timer0;

	//TODO: Generate error for incorrect number of parameters
	/*
  if (nrhs != 8)
    mexErrMsgTxt("incorrect number of arguments");
		*/
	
  vector< pair<double, double> > points;
  double angle_center, angle_range, angle_res, rho_center, rho_range, rho_res;

  int nlhs = 0;
//  double * xs = mxGetPr(prhs[XS]);
  double * xs = (double *)lua_touserdata(L, XS + 1);
  if ((xs == NULL) || !lua_islightuserdata(L, XS + 1)) {
    return luaL_error(L, "Input xs not light user data");
  } else nlhs++;
//  double * ys = mxGetPr(prhs[YS]);
  double * ys = (double *)lua_touserdata(L, YS + 1);
  if ((ys == NULL) || !lua_islightuserdata(L, YS + 1)) {
    return luaL_error(L, "Input ys not light user data");
  } else nlhs++;

//  int n_xs = mxGetNumberOfElements(prhs[XS]);
  int n_xs = sizeof(n_xs) / sizeof(double); 
//  int n_ys = mxGetNumberOfElements(prhs[YS]);
  int n_ys = sizeof(n_ys) / sizeof(double); 

  if (n_xs != n_ys)
    luaL_error(L, "xs and ys vectors must be have the same length");

  if (lua_isnoneornil(L, A_CENTER + 1)) 
    angle_center = 0;
  else {
    angle_center = luaL_checknumber(L, A_CENTER + 1);
    nlhs ++;
  }

  if (lua_isnoneornil(L, A_RANGE + 1)) 
    angle_range = 0;
  else {
    angle_range  = luaL_checknumber(L, A_RANGE + 1);
    nlhs ++;
  }

  if (lua_isnoneornil(L, A_RES + 1))
    angle_res = 0;
  else {
    angle_res = luaL_checknumber(L, A_RES + 1);
    nlhs ++;
  }
  
  if (lua_isnoneornil(L, R_CENTER + 1))
    rho_center = 0;
  else {
    rho_center = luaL_checknumber(L, R_CENTER + 1);
    nlhs ++;
  }
  
  if (lua_isnoneornil(L, R_RANGE + 1))
    rho_range = 0;
  else {
    rho_range = luaL_checknumber(L, R_RANGE + 1);
    nlhs ++;
  }
  
  if (lua_isnoneornil(L, R_RES + 1))
    rho_res = 0;
  else {
    rho_res = luaL_checknumber(L, R_RES + 1);
    nlhs ++;
  }

  int n_angles, n_rhos;

  //generate the angles
  GenerateRange(angle_center, angle_range, angle_res, angles, n_angles);

  //generate rhos
  GenerateRange(rho_center, rho_range, rho_res, rhos, n_rhos);

#ifdef HOUGH_TRANSFORM_API_DEBUG

  cout<<"number of points ="<<n_xs<<endl;
  
  cout<<"angles: ";
  for (int i=0; i<angles.size(); i++)
    cout<<angles[i]<<" ";
  cout<<endl;

  cout<<"rhos: ";
  for (int i=0; i<rhos.size(); i++)
    cout<<rhos[i]<<" ";
  cout<<endl;
#endif

  int n_out = n_angles*n_rhos;
  if (n_out > nh)
  {
    if (h_transform)
      delete [] h_transform;

    h_transform = new int[n_out];
    nh = n_out;
  }

  if (CalculateHoughTransform(xs, ys, n_xs, angles, rho_center, rho_range, rho_res, h_transform) != 0)
    luaL_error(L,"Could not compute Hough Transform"); 
  
  int out = 0;
  if (nlhs > 0)
  {    
    lua_createtable(L, n_out, 0);
    int * h_tr = h_transform;
    for (int cnt = 0; cnt < n_out; cnt ++) {
      lua_pushnumber(L, *(h_tr++));
      lua_rawseti(L, -2, cnt + 1);
    }
    lua_settable(L, -3);
    out ++;
  }

  if (nlhs>1)
  {
    lua_createtable(L, n_angles, 0);
    for (int cnt = 0; cnt < n_angles; cnt ++) {
      lua_pushnumber(L, angles[cnt]);
      lua_rawseti(L, -2, cnt + 1);
    }
    lua_settable(L, -3);
    out ++;
  }

  if (nlhs>2)
  {
    lua_createtable(L, n_rhos, 0);
    for (int cnt = 0; cnt < n_rhos; cnt ++) {
      lua_pushnumber(L, rhos[cnt]);
      lua_rawseti(L, -2, cnt + 1);
    }
    lua_settable(L, -3);
    out ++;
  }
  return out;
}

