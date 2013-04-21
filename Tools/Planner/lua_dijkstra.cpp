/*
  dijkstra planning library for lua
  
  Daniel D. Lee (ddlee@seas.upenn.edu), 4/2007
  Lua wrapper by Yida Zhang (yida@seas.upenn.edu), 4/2013
*/

#include <lua.hpp>

#ifdef TORCH
#ifdef __cplusplus
extern "C"
{
#endif
#include <luaT.h>
#include <TH/TH.h>
#include <TH/THGeneral.h>
#ifdef __cplusplus
}
#endif
#endif

#include <iostream>
#include <set>
#include <math.h>

using namespace std;

typedef pair<double, int> CostNodePair; // (cost, node)

typedef struct {
  int ioffset;
  int joffset;
  double distance;
} NeighborStruct;
NeighborStruct neighbors[] = {
  {-1,0, 1.0}, {1,0, 1.0}, {0,-1, 1.0}, {0,1, 1.0}, // 4-connected
  {-1,-1, sqrt(2)}, {1,-1, sqrt(2)}, {-1,1, sqrt(2)}, {1,1, sqrt(2)} // 8-connected
};
int nNeighbors = sizeof(neighbors)/sizeof(NeighborStruct);

static int lua_dijkstra_matrix(lua_State *L) {
  double *A = NULL;
#ifdef TORCH
  const char *tname = luaT_typename(L, 1);
  THDoubleTensor *costp = (THDoubleTensor *) luaT_checkudata(L, 1, tname);
#ifdef DEBUG
  std::cout << "Type Name " << tname << std::endl;
  std::cout << "Torch Dimension " << costp->nDimension << std::endl;
#endif
  THArgCheck(costp->nDimension == 2, 1, "tensor must have two dimensions");
  int size = costp->size[0] * costp->size[1];
  A = (double *)malloc(size * sizeof(double));
  // Get torchTensor data
  for (int r = 0; r < costp->size[0]; r++)
    for (int c = 0; c < costp->size[1]; c++)
      A[r * costp->size[1] + c] = (THTensor_fastGet2d(costp, r, c));
  int m = costp->size[0]; // number of rows;
  int n = costp->size[1]; // number of cols;

  int iGoal = luaL_optint(L, 2, 0) - 1; // 0-indexed nodes
  int jGoal = luaL_optint(L, 3, 0) - 1; // 0-indexed nodes
#endif

#ifdef DEBUG
  std::cout << size << std::endl;
  for (int i = 0; i < size; i++)
    std::cout << A[i] << " ";
  std::cout << std::endl;
#endif

  if (iGoal < 0) iGoal = 0;
  if (iGoal >= m-1) iGoal = m-1;

  if (jGoal < 0) iGoal = 0;
  if (jGoal >= n-1) iGoal = n-1;

  int indGoal = iGoal + m * jGoal; // linear index

  // Cost to go values
  double *D = (double *)malloc(m * n * sizeof(double));
  for (int i = 0; i < m*n; i++) D[i] = INFINITY;
  D[indGoal] = 0;

  // Priority queue implementation as STL set
  set<CostNodePair> Q; // Sorted set of (cost to go, node)
  Q.insert(CostNodePair(0, indGoal));

  while (!Q.empty()) {
    // Fetch closest node in queue
    CostNodePair top = *Q.begin();
    Q.erase(Q.begin());
    double c0 = top.first;
    int ind0 = top.second;

    // Array subscripts of node:
    int i0 = ind0 % m;
    int j0 = ind0 / m;
    // Iterate over neighbor nodes:
    for (int k = 0; k < nNeighbors; k++) {
      int i1 = i0 + neighbors[k].ioffset;
      if ((i1 < 0) || (i1 >= m)) continue;
      int j1 = j0 + neighbors[k].joffset;
      if ((j1 < 0) || (j1 >= n)) continue;
      int ind1 = m*j1+i1;

      double c1 = c0 + 0.5*(A[ind0]+A[ind1])*neighbors[k].distance;
      if (c1 < D[ind1]) {
	if (!isinf(D[ind1])) {
	  Q.erase(Q.find(CostNodePair(D[ind1],ind1)));
	}
	D[ind1] = c1;
	Q.insert(CostNodePair(D[ind1], ind1));
      }
    }
  }

#ifdef TORCH 
  THDoubleTensor *dp = THDoubleTensor_newWithSize2d(n, m);
  for (int r = 0; r < dp->size[0]; r++)
    for (int c = 0; c < dp->size[1]; c++)
      THTensor_fastSet2d(dp, r, c, D[r * dp->size[1] + c]);
  luaT_pushudata(L, dp, "torch.DoubleTensor");
#endif

  free(A);
  free(D);
  return 1;
}

static const luaL_Reg dijkstra_functions [] = {
  {"matrix", lua_dijkstra_matrix},
  {NULL, NULL}
};

static const luaL_Reg dijkstra_methods [] = {
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_dijkstra(lua_State *L) {
#if LUA_VERSION_NUM == 502
  luaL_newlib(L, dijkstra_functions);
#else
  luaL_register(L, "dijkstra", dijkstra_functions);
#endif
  return 1;
}
