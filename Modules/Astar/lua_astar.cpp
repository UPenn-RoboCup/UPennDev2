/*
  astar planning library for lua
  
  Daniel D. Lee (ddlee@seas.upenn.edu), 4/2007
  Lua wrapper by Yida Zhang (yida@seas.upenn.edu), 4/2013
*/

#include <lua.hpp>

#ifdef TORCH
#ifdef __cplusplus
extern "C"
{
#endif
#include <torch/luaT.h>
#include <torch/TH/TH.h>
#include <torch/TH/THGeneral.h>
#ifdef __cplusplus
}
#endif
#endif

#include <iostream>
#include <math.h>
#include <set>

#define STOP_FACTOR 1.1

using namespace std;

typedef pair<double, int> CostNodePair; // (cost, node)

typedef struct {
  int ioffset;
  int joffset;
  double distance;
} NeighborStruct;
NeighborStruct neighbors[] = {
  {1,0,1.0}, {2,1,sqrt(5)}, {1,1,sqrt(2)}, {1,2,sqrt(5)},
  {0,1,1.0}, {-1,2,sqrt(5)}, {-1,1,sqrt(2)}, {-2,1,sqrt(5)},
  {-1,0,1.0}, {-2,-1,sqrt(5)}, {-1,-1,sqrt(2)}, {-1,-2,sqrt(5)},
  {0,-1,1.0}, {1,-2,sqrt(5)}, {1,-1,sqrt(2)}, {2,-1,sqrt(5)},
};

static int lua_astar_graph(lua_State *L) {
  return 1;
}

/*
 [cost_to_go] = astar.nonholonomic(A, xya_goal, xya_start, [nNeighbors]);
  where positive costs are given in matrix A with nNeighbors different orientations.
 */
static int lua_astar_nonholonomic(lua_State *L) {
  double *A = NULL;
  double *prGoal = NULL;
  double *prStart = NULL;

#ifdef TORCH
  // check input A
  const char *tname = luaT_typename(L, 1);
  THDoubleTensor *costp = (THDoubleTensor *) luaT_checkudata(L, 1, tname);
#ifdef DEBUG
  std::cout << "Type Name " << tname << std::endl;
  std::cout << "Torch Dimension " << costp->nDimension << std::endl;
#endif
  THArgCheck(costp->nDimension == 2, 1, "cost tensor must have two dimensions");
  int size = costp->size[0] * costp->size[1];
  A = (double *)malloc(size * sizeof(double));
  // Get torchTensor data
  for (int r = 0; r < costp->size[0]; r++)
    for (int c = 0; c < costp->size[1]; c++)
      A[r * costp->size[1] + c] = (THTensor_fastGet2d(costp, r, c));
  int m = costp->size[0]; // number of rows;
  int n = costp->size[1]; // number of cols;

  // check input prGoal
  const char *goal_name = luaT_typename(L, 1);
  THDoubleTensor *goalp = (THDoubleTensor *) luaT_checkudata(L, 1, goal_name);
#ifdef DEBUG
  std::cout << "Type Name " << goal_name << std::endl;
  std::cout << "Torch Dimension " << goalp->nDimension << std::endl;
#endif
  THArgCheck(goalp->nDimension == 1, 1, "goal must have one dimension");
  THArgCheck(goalp->size[0] == 3, 1, "Goal should be (xgoal, ygoal, agoal)");
  prGoal = (double *)malloc(3 * sizeof(double));
  for (int i = 0; i < 3; i++)
    prGoal[i] = THTensor_fastGet1d(goalp, i);

  // check input prStart
  const char *start_name = luaT_typename(L, 1);
  THDoubleTensor *startp = (THDoubleTensor *) luaT_checkudata(L, 1, start_name);
#ifdef DEBUG
  std::cout << "Type Name " << start_name << std::endl;
  std::cout << "Torch Dimension " << startp->nDimension << std::endl;
#endif
  THArgCheck(startp->nDimension == 1, 1, "start must have one dimension");
  THArgCheck(startp->size[0] == 3, 1, "Goal should be (xstart, ystart, astart)");
  prStart = (double *)malloc(3 * sizeof(double));
  for (int i = 0; i < 3; i++)
    prStart[i] = THTensor_fastGet1d(startp, i);

  int nNeighbors = luaL_optint(L, 4, 16); // default 16 neighbors
#endif

  int iGoal = round(prGoal[0]-1); // 0-indexing
  if (iGoal < 0) iGoal = 0;
  if (iGoal > m-1) iGoal = m-1;
  int jGoal = round(prGoal[1]-1);
  if (jGoal < 0) jGoal = 0;
  if (jGoal > n-1) jGoal = n-1;
  int aGoal = round(nNeighbors/(2*M_PI)*prGoal[2]);
  aGoal = aGoal % nNeighbors;
  if (aGoal < 0) aGoal += nNeighbors;
  int indGoal = iGoal + m*jGoal + m*n*aGoal; // linear index

  int iStart = round(prStart[0]-1);
  if (iStart < 0) iStart = 0;
  if (iStart > m-1) iStart = m-1;
  int jStart = round(prStart[1]-1);
  if (jStart < 0) jStart = 0;
  if (jStart > n-1) jStart = n-1;
  int aStart = round(nNeighbors/(2*M_PI)*prStart[2]);
  aStart = aStart % nNeighbors;
  if (aStart < 0) aStart += nNeighbors;
  int indStart = iStart + m*jStart + m*n*aStart; // linear index

  // Cost to go values
  double *D = (double *) malloc(nNeighbors * m * n * sizeof(double));
  for (int i = 0; i < nNeighbors*m*n; i++) D[i] = INFINITY;
  D[indGoal] = 0;

  // Priority queue implementation as STL set
  set<CostNodePair> Q; // Sorted set of (cost to go, node)
  Q.insert(CostNodePair(0, indGoal));

  int nNode = 0;
  while (!Q.empty()) {
    nNode++;
    // Fetch closest node in queue
    CostNodePair top = *Q.begin();
    Q.erase(Q.begin());
    double f0 = top.first;
    int ind0 = top.second;
    double c0 = D[ind0];

    // Short circuit computation if path to start has been found:
    if (f0 > STOP_FACTOR*D[indStart]) break;

    // Array subscripts of node:
    int a0 = ind0 / (m*n);
    int ij0 = ind0 - a0*m*n;
    int j0 = ij0 / m;
    int i0 = ij0 % m;
    // Iterate over neighbor nodes:
    for (int ashift = -1; ashift <= +1; ashift++) {
      int a1 = (a0+nNeighbors+ashift) % nNeighbors; // Non-negative heading index

      int ioffset = neighbors[a1].ioffset;
      int joffset = neighbors[a1].joffset;

      int i1 = i0 - ioffset;
      if ((i1 < 0) || (i1 >= m)) continue;
      int j1 = j0 - joffset;
      if ((j1 < 0) || (j1 >= n)) continue;

      double cost = A[ij0];
      int koffset = floor(neighbors[a1].distance);
      for (int k = 1; k < koffset; k++) {
	      int ik = i0 - k*ioffset/koffset;
      	int jk = j0 - k*joffset/koffset;
      	cost += A[m*jk+ik];
      }
      cost /= koffset;

      double c1 = c0 + cost*neighbors[a1].distance;
      // Heuristic cost to start:
      double h1 = sqrt((iStart-i1)*(iStart-i1)+(jStart-j1)*(jStart-j1));
      // Estimated total cost:
      double f1 = c1 + h1;

      int ind1 = m*j1 + i1 + m*n*a1;
      if (c1 < D[ind1]) {
      	if (!isinf(D[ind1])) {
      	  Q.erase(CostNodePair(D[ind1]+h1,ind1));
      	}
      	D[ind1] = c1;
      	Q.insert(CostNodePair(f1, ind1));
      }
      
    }
  }

#ifdef DEBUG
  std::cout << "Astar: nNode = " << nNode << ", queue = " << Q.size() << std::endl;
#endif

#ifdef TORCH 
  THDoubleTensor *dp = THDoubleTensor_newWithSize3d(n, m, nNeighbors);
  for (int s = 0; s < dp->size[2]; s++)
    for (int r = 0; r < dp->size[0]; r++)
      for (int c = 0; c < dp->size[1]; c++)
        THTensor_fastSet3d(dp, r, c, s, D[s * dp->size[1] * dp->size[0] + r * dp->size[1] + c]);
  luaT_pushudata(L, dp, "torch.DoubleTensor");
#endif

  free(A);
  free(D);
  return 1;
}

static const luaL_Reg astar_functions [] = {
  {"nonholomonic", lua_astar_nonholonomic},
  {"graph", lua_astar_graph},
  {NULL, NULL}
};

static const luaL_Reg astar_methods [] = {
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_astar(lua_State *L) {
#if LUA_VERSION_NUM == 502
  luaL_newlib(L, astar_functions);
#else
  luaL_register(L, "astar", astar_functions);
#endif

  return 1;
}
