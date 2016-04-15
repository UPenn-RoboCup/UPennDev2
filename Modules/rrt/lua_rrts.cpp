#include <lua.hpp>

#include <ctime>
#include <math.h>

#include "rrts.hpp"
#include "system_single_integrator.h"

#define TIMING

// 20 works @ dim 5
// 30 works @ dim 6
// 36 works @dim 7 (Found on iteration 63877 in 178seconds @ Gamma 2)
// With the initial joint angle suff w/ modified search zone: 30 degrees works
#define DEGREE_TOLERANCE 20//36

using namespace RRTstar;
using namespace SingleIntegrator;
using namespace std;

extern double* xyzG;
double* xyzG = new double[6];

typedef Planner<State,Trajectory,System> planner_t;
typedef Vertex<State,Trajectory,System> vertex_t;

static int lua_rrts_plan(lua_State *L) {
    
  // This parameter should be larger than 1.5 for asymptotic 
  //   optimality. Larger values will weigh on optimization 
  //   rather than exploration in the RRT* algorithm. Lower 
  //   values, such as 0.1, should recover the RRT.
  const double gamma = luaL_optnumber(L, 6, 1.5);
  const int nIterations = luaL_optint(L, 7, 10000);
  
  // Initialize the planner
  planner_t rrts;
  
  // Create the dynamical system that checks obstacles and closeness
  System system;
  
  
  // Initial pose
  luaL_checktype(L, 1, LUA_TTABLE);
#if LUA_VERSION_NUM == 502
		int nDim = lua_rawlen(L, 1);
#else
		int nDim = lua_objlen(L, 1);
#endif
    system.setNumDimensions(nDim);
    
    // fk pose
    luaL_checktype(L, 2, LUA_TTABLE);
  #if LUA_VERSION_NUM == 502
  		if(6 != lua_rawlen(L, 2)){ return luaL_error(L, "Bad fk dimensions"); }
  #else
      if(6 != lua_objlen(L, 2)){ return luaL_error(L, "Bad fk dimensions"); }
  #endif
      for(int i = 0;i<6;i++){
        lua_rawgeti(L, 2, i+1);
        xyzG[i] = lua_tonumber(L, -1);
        lua_pop(L, 1);
      }
    
    // TODO: This should be optional for goal biasing...
  // Guessed pose
  luaL_checktype(L, 3, LUA_TTABLE);
#if LUA_VERSION_NUM == 502
		if(nDim != lua_rawlen(L, 3)){ return luaL_error(L, "Bad goal dimensions"); }
#else
    if(nDim != lua_objlen(L, 3)){ return luaL_error(L, "Bad goal dimensions"); }
#endif
  // Define the goal region
  system.regionGoal.setNumDimensions(nDim);  
	for (int i = 0; i < nDim; i++) {
    lua_rawgeti(L, 3, i+1);
		system.regionGoal.center[i] = lua_tonumber(L, -1);
    system.regionGoal.size[i] = 2 * DEGREE_TOLERANCE * M_PI / 180;
		lua_pop(L, 1);
	}
    
    
  // qMid
  luaL_checktype(L, 4, LUA_TTABLE);
  // qRange
  luaL_checktype(L, 5, LUA_TTABLE);
  #if LUA_VERSION_NUM == 502
  		if(nDim != lua_rawlen(L, 4)){ return luaL_error(L, "Bad mid dimensions"); }
      if(nDim != lua_rawlen(L, 5)){ return luaL_error(L, "Bad range dimensions"); }
  #else
      if(nDim != lua_objlen(L, 4)){ return luaL_error(L, "Bad mid dimensions"); }
      if(nDim != lua_objlen(L, 5)){ return luaL_error(L, "Bad range dimensions"); }
  #endif

  // Define the operating region (Mid, Size)
  system.regionOperating.setNumDimensions(nDim);
  for(int i = 0;i<nDim;i++){
    lua_rawgeti(L, 4, i+1);
    system.regionOperating.center[i] = lua_tonumber(L, -1);
    lua_pop(L, 1);
    lua_rawgeti(L, 5, i+1);
    system.regionOperating.size[i] = lua_tonumber(L, -1);
    lua_pop(L, 1);
  }
    
  // Add the system to the planner
  rrts.setSystem (system);
  
  // Set up the root vertex
  vertex_t &root = rrts.getRootVertex();  
  State &rootState = root.getState();
  
  // Get the initial state
	for (int i = 0; i < nDim; i++) {
    lua_rawgeti(L, 1, i+1);
		rootState[i] = lua_tonumber(L, -1);
		lua_pop(L, 1);
	}
    
  // Initialize the planner
  rrts.initialize();
  rrts.setGamma(gamma);

    double* ostate = NULL;
    double* pstate = NULL;
    
#ifdef CSV
    FILE * pFile;
    pFile = fopen ("/tmp/samples.csv","w");
    if(pFile != NULL){
      ostate = new double[nDim];
      pstate = new double[nDim];
    }
#endif
    
#ifdef TIMING
    clock_t start = clock();
#endif
    
    // Run the algorithm for 10000 iteartions
    for (int i = 0; i < nIterations; i++){
#ifdef TIMING
      if(i%1000 == 0){
        clock_t now = clock();
        printf("Completion: %4.2f, %5.2f sec\n", i/(double)nIterations, ((double)(now-start))/CLOCKS_PER_SEC);
      }
#endif

      rrts.iteration(ostate, pstate);

#ifdef CSV
      rrts.iteration(ostate, pstate);
      if (pFile!=NULL){
        for(int i=0;i<nDim;i++){
          fprintf(pFile, "%.2f ", pstate[i]);
        }
        for(int i=0;i<nDim;i++){
          fprintf(pFile, "%.2f ", ostate[i]);
        }
        fprintf(pFile, "\n");
      }
#endif
      
    }

#ifdef TIMING
    clock_t finish = clock();
    printf("TimeL %.2f\n", ((double)(finish-start))/CLOCKS_PER_SEC);
    fflush(stdout);
#endif

#ifdef CSV
    if (pFile!=NULL)
    {
      fclose (pFile);
    }
#endif

  std::list<double*> finalTraj;
  int ret = rrts.getBestTrajectory(finalTraj);

  // No path found
  if(!ret){ return 0; }
  
  lua_createtable(L, finalTraj.size(), 0);
  //printf("Num path: %d\n", finalTraj.size());
  int j = 0;
  for (std::list<double*>::iterator it = finalTraj.begin(); it != finalTraj.end(); it++){
    double* coord = *it;
    lua_createtable(L, nDim, 0);
    for (int i = 0; i < nDim; i++) {
      lua_pushnumber(L, coord[i]);
      lua_rawseti(L, -2, i+1);
    }
    lua_rawseti(L, -2, ++j);
  }
  
  return 1;
}

static int lua_rrts_smooth(lua_State *L) {
  return 0;
}

static const struct luaL_Reg rrts_lib [] = {
  {"plan", lua_rrts_plan},
  {"smooth", lua_rrts_smooth},
  {NULL, NULL}
};

extern "C" int luaopen_rrts(lua_State *L) {
#if LUA_VERSION_NUM == 502
  luaL_newlib(L, rrts_lib);
#else
  luaL_register(L, "rrts", rrts_lib);
#endif
  
  // Seed the random bits
  srand (time(NULL));
  //srand(123);
  
  return 1;
}


