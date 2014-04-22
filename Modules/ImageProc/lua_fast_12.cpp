/*
 * Daniel D. Lee <ddlee@seas.upenn.edu> 01/2013
 *
 * Lua module to provide image processing functions
*/

#include <string.h>
#include "fast_12.h"
#include <lua.hpp>

int lua_fast_12(lua_State *L) {
  size_t len;
  const byte *im;
  if (lua_type(L, 1) == LUA_TLIGHTUSERDATA) {
    im = (const byte *) lua_topointer(L, 1);
  }
  else if (lua_type(L, 1) == LUA_TSTRING) {
    im = (const byte *)lua_tolstring(L, 1, &len);
  }
  else {
    luaL_error(L, "Invalid image type");
  }
  int width = luaL_checkint(L, 2);
  int height = luaL_checkint(L, 3);
  int threshold = luaL_optint(L, 4, 1);
  int xstride = luaL_optint(L, 5, 1);
  int ystride = luaL_optint(L, 6, width);
  int offset = luaL_optint(L, 7, 0);

  int *xc, *yc;
  int nc = fast12_corner(im+offset,
			 width, height,
			 threshold,
			 xstride, ystride,
			 &xc, &yc);
  
  // Return lua table {x1,y1,x2,y2,...}
  lua_createtable(L, 2*nc, 0);
  for (int i = 0; i < nc; i++) {
    lua_pushinteger(L, xc[i]);
    lua_rawseti(L, -2, 2*i+1);
    lua_pushinteger(L, yc[i]);
    lua_rawseti(L, -2, 2*i+2);
  }
  return 1;
}

//static const struct luaL_reg imageproc_functions[] = {
//  {"fast12", imageproc_fast12},
//  {NULL, NULL}
//};
//
//#ifdef __cplusplus
//extern "C"
//#endif
//int luaopen_imageproc (lua_State *L) {
//  luaL_register(L, "imageproc", imageproc_functions);
//  return 1;
//}
