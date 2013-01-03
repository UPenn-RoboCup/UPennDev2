/* 
(c) 2013 Dan Lee, Alex Kushlyev, Steve McGill, Yida Zhang
ddlee@seas.upenn.edu, smcgill3@seas.upenn.edu
University of Pennsylvania
*/

/*
  s = binStats(x, y, n);
  Returns struct containing statistics of vector y
  binned according to round(x), up to optional size n

  mex -O binStats.cc
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

#include <math.h>
#include <float.h>
#include <vector>
#include <string.h>

std::vector<int> Count;
std::vector<double> sumY, sumYY, maxY, minY;
std::vector<double> nbin;


int lua_binStats(lua_State *L)
{
  double *prX = (double *) lua_touserdata(L, 1); 
  if ((prX == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "Input X not light user data");
  }

  double *prY = (double *) lua_touserdata(L, 2); 
  if ((prY == NULL) || !lua_islightuserdata(L, 2)) {
    return luaL_error(L, "Input Y not light user data");
  }

  int nX = sizeof(prX) / sizeof(double);
  int nY = sizeof(prY) / sizeof(double);
  
  if (nX != nY)
    luaL_error(L, "Number of elements in inputs should match");

  int n = 0;
  bool thirdvar = false;
    // Find max value of x:
  if (lua_isnoneornil(L, 3)) {
    for (int i = 0; i < nX; i++) {
      if (prX[i] > n) n = round(prX[i]);
    }
  } else {
    n = luaL_checkint(L, 3);
    thirdvar = true;
  }

  // Initialize statistics vectors:
  Count.resize(n);
  sumY.resize(n);
  sumYY.resize(n);
  maxY.resize(n);
  minY.resize(n);
  nbin.resize(nX,-1);
  for (int i = 0; i < n; i++) {
    Count[i] = 0;
    sumY[i] = 0;
    sumYY[i] = 0;
    maxY[i] = -__FLT_MAX__;
    minY[i] = __FLT_MAX__;
  }

  for (int i = 0; i < nX; i++) {
    int j = round(prX[i]) - 1;
    if ((j >= 0) && (j < n)) {
      Count[j]++;
      sumY[j] += prY[i];
      sumYY[j] += prY[i]*prY[i];
      if (prY[i] > maxY[j]) maxY[j] = prY[i];
      if (prY[i] < minY[j]) minY[j] = prY[i];
      nbin[i] = (double)j+1;
    }
  }

  const char *fields[] = {"count", "mean", "std", "max", "min","nbin"};
  const int nfields = sizeof(fields)/sizeof(*fields);
  double mean = 0, std = 0, max = 0, min = 0;
  lua_createtable(L, n, 0);
  for (int i = 0; i < n; i++) { 
    lua_createtable(L, 0, nfields - 1);
    
    lua_pushstring(L, fields[0]);
    lua_pushnumber(L, Count[i]);
    lua_settable(L, -3);

    mean = 0; std = 0; max = 0; min = 0;
    if (Count[i] > 0) {
      mean = sumY[i]/Count[i];
      std = sqrt((sumYY[i]-sumY[i]*sumY[i]/Count[i])/Count[i]);
      max = maxY[i];
      min = minY[i];
    }

    lua_pushstring(L, fields[1]);
    lua_pushnumber(L, mean);
    lua_settable(L, -3);

    lua_pushstring(L, fields[2]);
    lua_pushnumber(L, std);
    lua_settable(L, -3);    

    lua_pushstring(L, fields[3]);
    lua_pushnumber(L, max);
    lua_settable(L, -3);     

    lua_pushstring(L, fields[4]);
    lua_pushnumber(L, min);
    lua_settable(L, -3); 

  }

  if (thirdvar) {
    lua_createtable(L, nX, 0);
    for (int cnt = 0; cnt < nX; cnt++) {
      lua_pushnumber(L, nbin[cnt]);
      lua_rawseti(L, -2, cnt + 1);
    }
    lua_settable(L, -3);
    return 2;
  } else {
    return 1;
  }
}
