/* 
(c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
University of Pennsylvania
*/

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
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#define MAX_NUM_CELLS 3000000

//Bresenham's line algorithm
int lua_getMapCellsFromRay(lua_State *L)
{
    static std::vector<double> xio;
    static std::vector<double> yio;

    if (!xio.size())
    {
      xio.resize(MAX_NUM_CELLS);
      yio.resize(MAX_NUM_CELLS);
    }

    int x0t = luaL_checkint(L, 1);
    int y0t = luaL_checkint(L, 2);

    double * xis = (double *) lua_touserdata(L, 3); 
    if ((xis == NULL) || !lua_islightuserdata(L, 3)) {
      return luaL_error(L, "Input Xis not light user data");
    }
  
    double * yis = (double *) lua_touserdata(L, 4); 
    if ((yis == NULL) || !lua_islightuserdata(L, 4)) {
      return luaL_error(L, "Input Yis not light user data");
    }
    int nPoints = sizeof(xis) / sizeof(double);

    std::vector<double> pxio = xio;
    std::vector<double> pyio = yio;

    for (int ii=0; ii<nPoints; ii++)
    {
      int x0 = x0t;
      int y0 = y0t;

      int x1 = (int)*xis++;
      int y1 = (int)*yis++;    

      bool steep = abs(y1 - y0) > abs(x1 - x0);
      if(steep){
          int temp = x0;
          x0 = y0;
          y0 = temp;
          temp = x1;
          x1 = y1;
          y1 = temp;
      }
      if(x0 > x1){
          int temp = x0;
          x0 = x1;
          x1 = temp;
          temp = y0;
          y0 = y1;
          y1 = temp;
      }
      int deltax = x1 - x0;
      int deltay = abs(y1 - y0);
      float error = deltax / 2;
      int y = y0;
      int ystep;
      if(y0 < y1)
          ystep = 1;
      else
          ystep = -1;

      //int num_pts = x1-x0-1;
      //plhs[0] = mxCreateDoubleMatrix(num_pts, 2, mxREAL);
      //double* cells = mxGetPr(plhs[0]);
      
      if(steep){
          for(int x=x0; x<(x1); x++){
              pxio.push_back(y);
              pyio.push_back(x);
              //cells[x-x0] = y;
              //cells[num_pts + x-x0] = x;
              error = error - deltay;
              if(error < 0){
                  y += ystep;
                 error += deltax;
              }
          }
      }
      else{
          for(int x=x0; x<(x1); x++){
              pxio.push_back(x);
              pyio.push_back(y);
              //cells[x-x0] = x;
              //cells[num_pts + x-x0] = y;
              error = error - deltay;
              if(error < 0){
                  y += ystep;
                 error += deltax;
              }
          }
      }
    }

    int numCells = pxio.size() - xio.size();

    printf("generated %d cells\n",numCells);

    lua_createtable(L, numCells, 0);
    for (int cnt = 0; cnt < numCells; cnt ++) {
      lua_pushnumber(L, xio[cnt]);
      lua_rawseti(L, -2, cnt + 1);
    }
    lua_settable(L, -3);

    lua_createtable(L, numCells, 0);
    for (int cnt = 0; cnt < numCells; cnt ++) {
      lua_pushnumber(L, yio[cnt]);
      lua_rawseti(L, -2, cnt + 1);
    }
    lua_settable(L, -3);

    return 2;

}
