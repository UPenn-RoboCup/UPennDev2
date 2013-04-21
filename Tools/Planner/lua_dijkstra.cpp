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
#ifdef __cplusplus
}
#endif
#endif

#include <iostream>
#include <vector>

using namespace std;

// dimensions, sizes
//template<>
//{
//}

static int lua_dijkstra_matrix(lua_State *L) {
#ifdef TORCH
  const char *tname = luaT_typename(L, 1);
  THDoubleTensor *costp = (THDoubleTensor *) luaT_checkudata(L, 1, tname);

  std::cout << "Type Name " << tname << std::endl;
  std::cout << "Torch Dimension " << costp->nDimension << std::endl;

  vector<double> data;
  switch (costp->nDimension) {
    case 1:
      for (int i = 0; i < costp->size[0]; i++)
        data.push_back(THTensor_fastGet1d(costp, i));
      break;
    case 2:
      for (int i = 0; i < costp->size[0]; i++)
        for (int j = 0; j < costp->size[1]; j++)
          data.push_back(THTensor_fastGet2d(costp, i, j));
      break;
    case 3:
      for (int i = 0; i < costp->size[0]; i++)
        for (int j = 0; j < costp->size[1]; j++)
          for (int k = 0; k < costp->size[2]; k++)
            data.push_back(THTensor_fastGet3d(costp, i, j, k));
      break;
    case 4:
      for (int i = 0; i < costp->size[0]; i++)
        for (int j = 0; j < costp->size[1]; j++)
          for (int k = 0; k < costp->size[2]; k++)
            for (int l = 0; l < costp->size[3]; l++)
              data.push_back(THTensor_fastGet4d(costp, i, j, k, l));
      break;
    default:
      break;
  }
#endif
  std::cout << data.size() << std::endl;
  for (int i = 0; i < data.size(); i++)
    std::cout << data[i] << " ";
  std::cout << std::endl;
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
