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
#include <vector>
#include <utility>
#include <set>
#include <math.h>

#define TURN_COST 1.2

#define STOP_FACTOR 1.1

using namespace std;

typedef pair<double, int> CostNodePair; // (cost, node)

typedef struct {
  int ioffset;
  int joffset;
  double distance;
} NeighborStruct;
NeighborStruct neighbors[] = {
  {-1,0, 1.0}, {1,0, 1.0}, {0,-1, 1.0}, {0,1, 1.0}, // 4-connected
  {-1,-1, sqrt(2)}, {1,-1, sqrt(2)}, {-1,1, sqrt(2)}, {1,1, sqrt(2)}, // 8-connected
  {2,1,sqrt(5)}, {1,2,sqrt(5)}, {-1,2,sqrt(5)}, {-2,1,sqrt(5)}, 
  {-2,-1,sqrt(5)}, {-1,-2,sqrt(5)}, {1,-2,sqrt(5)}, {2,-1,sqrt(5)}, // 16-connected
};

/*
  cost_to_go = dijkstra.matrix(A, i_goal, j_goal, [nNeighbors])

  where positive costs are given in matrix A with nNeighbors,
  and (i_goal, j_goal) is the goal node.
*/ 
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

  int nNeighbors = luaL_optint(L, 4, 8);
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

/*
  [cost_to_go] = dijkstra_nonholonomic(A, xya_goal, xya_start, [nNeighbors]);
  where positive costs are given in matrix A with nNeighbors different orientations.
*/ 
static int lua_dijkstra_nonholonomic(lua_State *L) {
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

  /* Due to hopping in 16 neighbor pattern, need to seed
     a cluster of goal states--using 4 corners with (iGoal, jGoal)
     being one corner */
  int iGoal = floor(prGoal[0]-1); // 0-indexing
  if (iGoal < 0) iGoal = 0;
  if (iGoal > m-2) iGoal = m-2;
  int jGoal = floor(prGoal[1]-1);
  if (jGoal < 0) jGoal = 0;
  if (jGoal > n-2) jGoal = n-2;
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

  // Priority queue implementation as STL set
  set<CostNodePair> Q; // Sorted set of (cost to go, node)
  // Seeding goal states
  D[indGoal] = 0;
  Q.insert(CostNodePair(0, indGoal));
  D[indGoal+1] = 0;
  Q.insert(CostNodePair(0, indGoal+1));
  D[indGoal+m] = 0;
  Q.insert(CostNodePair(0, indGoal+m));
  D[indGoal+m+1] = 0;
  Q.insert(CostNodePair(0, indGoal+m+1));

  int nNode = 0;
  while (!Q.empty()) {
    nNode++;
    // Fetch closest node in queue
    CostNodePair top = *Q.begin();
    Q.erase(Q.begin());
    double c0 = top.first;
    int ind0 = top.second;

    // Short circuit computation if path to start has been found:
    if (c0 > STOP_FACTOR*D[indStart]) break;

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
      for (int k = 1; k <= koffset; k++) {
	      int ij = ij0 - k*(m*joffset + ioffset)/koffset;
	      if (A[ij] > cost) cost = A[ij];
      }
      
      if (ashift != 0) cost *= TURN_COST;

      double c1 = c0 + cost*neighbors[a1].distance;

      int ind1 = i1 + m*j1 + m*n*a1;
      if (c1 < D[ind1]) {
      	if (!isinf(D[ind1])) {
	        Q.erase(CostNodePair(D[ind1],ind1));
	      }
      	D[ind1] = c1;
      	Q.insert(CostNodePair(D[ind1], ind1));
      }
    }
  }

#ifdef TORCH 
  THDoubleTensor *dp = THDoubleTensor_newWithSize3d(n, m, nNeighbors);
  for (int s = 0; s < dp->size[2]; s++)
    for (int r = 0; r < dp->size[0]; r++)
      for (int c = 0; c < dp->size[1]; c++)
        THTensor_fastSet3d(dp, r, c, s, D[s * dp->size[1] * dp->size[0] + r * dp->size[1] + c]);
  luaT_pushudata(L, dp, "torch.DoubleTensor");
#endif

#ifdef DEBUG
  std::cout << "Dijkstra: nNode = " << nNode << ", queue = " << Q.size() << std::endl;
#endif

  free(A);
  free(D);
  return 1;
}

/*
  [cost_to_go, next_index] = dijkstra_graph(A, goal_index)
  where connected edge costs i->j are given as positive entries
  in the sparse adjacency matrix A(i,j).
  
  dist is the cost to go to the goal node
  next_index containts the next index to traverse

 */
typedef pair<double,int> di;  // (cost, from node)
typedef vector<di> vdi; // edge list for a single to node
typedef vector<vdi> vvdi; // vector of edge lists to all nodes

static int lua_dijkstra_graph(lua_State *L) {
  return 0;
}

const int ioffset[] = {-1, 1, 0, 0, -1, 1, -1, 1};
const int joffset[] = {0, 0, -1, 1, -1, -1, 1, 1};
const double doffset[] = {1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2)};
const double eps = 0.0000001;

static int lua_dijkstra_path(lua_State *L) {
    double *A = NULL;
    double *C = NULL;
#ifdef TORCH
    const char *tname = luaT_typename(L, 1);
    THDoubleTensor *Ap = (THDoubleTensor *) luaT_checkudata(L, 1, tname);
    THArgCheck(Ap->nDimension == 2, 1, "dijkstra matrix must have two dimensions");
    
    int size = Ap->size[0] * Ap->size[1];
    A = (double *)malloc(size * sizeof(double));
    // Get torchTensor data
    for (int r = 0; r < Ap->size[0]; r++)
      for (int c = 0; c < Ap->size[1]; c++)
        A[r * Ap->size[1] + c] = (THTensor_fastGet2d(Ap, r, c));
    int m = Ap->size[0]; // number of rows;
    int n = Ap->size[1]; // number of cols;
    
    tname = luaT_typename(L, 2);
    THDoubleTensor *costp = (THDoubleTensor *) luaT_checkudata(L, 2, tname);
    THArgCheck(costp->nDimension == 2, 1, "cost matrix must have two dimensions");

    size = costp->size[0] * costp->size[1];
    C = (double *)malloc(size * sizeof(double));
    // Get torchTensor data
    for (int r = 0; r < costp->size[0]; r++)
      for (int c = 0; c < costp->size[1]; c++)
        C[r * costp->size[1] + c] = (THTensor_fastGet2d(costp, r, c));
 
    int istart = luaL_optint(L, 3, 0) - 1; // 0-indexed nodes
    int jstart = luaL_optint(L, 4, 0) - 1; // 0-indexed nodes
    
#endif
    std::vector<int> ipath;
    std::vector<int> jpath;
    ipath.push_back(istart);
    jpath.push_back(jstart);

    int i0 = 0, j0 = 0, i1 = 0, j1 = 0;
    double d1 = 0;
    std::vector<int> iarray;
    std::vector<int> jarray;
    int array_size = sizeof(ioffset) / sizeof(int);
    iarray.resize(array_size);
    jarray.resize(array_size);
    std::vector<int> valid_idx;
    while (1) {
        i0 = ipath[ipath.size() - 1];
        j0 = jpath[jpath.size() - 1];
        int ind0 = i0 * n + j0;

        if (A[i0 * n + j0] < eps) break;
        
        valid_idx.clear();
        for (int cnt = 0; cnt < array_size; cnt++) {
            iarray[cnt] = i0 + ioffset[cnt];
            jarray[cnt] = j0 + joffset[cnt];
            if ((iarray[cnt] >= 0) && (iarray[cnt] < m) 
                && (jarray[cnt] >=0) && (jarray[cnt] < n))
                valid_idx.push_back(cnt);
        }
        double min_a = 10000000;
        int min_idx = 0;
        for (int cnt = 0; cnt < valid_idx.size(); cnt ++) {
            i1 = iarray[valid_idx[cnt]];
            j1 = jarray[valid_idx[cnt]];
            d1 = doffset[valid_idx[cnt]];
            int ind1 = i1 * n + j1;

            double a1 = A[ind1] + 0.5 * d1 * (C[ind1] + C[ind0]);
            if (a1 < min_a) {
                min_a = a1;
                min_idx = valid_idx[cnt];
            }
        }
        ipath.push_back(iarray[min_idx]);
        jpath.push_back(jarray[min_idx]);
    }

#ifdef TORCH 
  THDoubleTensor *ipathp = THDoubleTensor_newWithSize1d(ipath.size());
  for (int i = 0; i < ipath.size(); i++)
        THTensor_fastSet1d(ipathp, i, ipath[i]);
  luaT_pushudata(L, ipathp, "torch.DoubleTensor");

  THDoubleTensor *jpathp = THDoubleTensor_newWithSize1d(jpath.size());
  for (int i = 0; i < jpath.size(); i++)
        THTensor_fastSet1d(jpathp, i, jpath[i]);
  luaT_pushudata(L, jpathp, "torch.DoubleTensor");
#endif

    free(A);
    free(C);
    return 2;
}

static const luaL_Reg dijkstra_functions [] = {
  {"matrix", lua_dijkstra_matrix},
  {"nonholomonic", lua_dijkstra_nonholonomic},
  {"graph", lua_dijkstra_graph},
  {"path", lua_dijkstra_path},
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
