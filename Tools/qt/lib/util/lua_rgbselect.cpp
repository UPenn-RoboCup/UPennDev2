/*
  y = rgbselect.select(data,w,h,ptx,pty,threshold);

  lua file to flood select region of RGB image x;
  Modified from Mex

  Daniel D. Lee, 03/2006
  <ddlee@seas.upenn.edu>
  Yida Zhang, 02/2013
  <yida@seas.upenn.edu>
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

#include <deque>
#include <stdint.h>
#include <iostream>
#include <math.h>

using namespace std;
typedef unsigned char uint8;

class Node {
public:
  Node(int x0=0, int y0=0) : x(x0), y(y0) {}
  int x,y;
};

static int lua_select(lua_State *L) {
  const int MAX_COLS = 65536;

  int iy0 = luaL_checkint(L, 4);

  int ix0 = luaL_checkint(L, 5);

  double threshold = 16;
  if (lua_isnoneornil(L, 6) == 0) 
    threshold = luaL_checknumber(L, 6);

  uint8_t *in_ptr = (uint8_t *)lua_topointer(L, 1);
  if (in_ptr == NULL) {
    return luaL_error(L, "RGB Input needed");
  }
  
  int ny = luaL_checkint(L, 2);
  int nx = luaL_checkint(L, 3);

  // Create output argument
  uint8_t out_ptr[nx * ny];

  // Construct array pointers
  uint8_t r_in[nx][ny], g_in[nx][ny], b_in[nx][ny];
  uint8_t *label[MAX_COLS];

  int pxcounter = 0;
  for (int ix = 0; ix < nx; ix++) {
    for (int iy = 0; iy < ny; iy++) {
      r_in[ix][iy] = in_ptr[pxcounter];
      g_in[ix][iy] = in_ptr[pxcounter + 1];
      b_in[ix][iy] = in_ptr[pxcounter + 2];
      pxcounter += 3;
    }
  }

  for (int ix = 0; ix < nx; ix++) {
    label[ix] = out_ptr + ix*ny;
  }

  double rmean = r_in[ix0][iy0];
  double gmean = g_in[ix0][iy0];
  double bmean = b_in[ix0][iy0];
  int nmean = 0;

  deque<Node> stack;
  stack.push_back(Node(ix0,iy0));

  while (!stack.empty()) {
    Node node = stack.front();
    stack.pop_front();

    int ix = node.x;
    int iy = node.y;

    // Skip test if pixel was already checked
    if (label[ix][iy] > 0) continue;

    double r_diff = r_in[ix][iy] - rmean;
    double g_diff = g_in[ix][iy] - gmean;
    double b_diff = b_in[ix][iy] - bmean;

    if ((fabs(r_diff) < threshold) &&
	(fabs(g_diff) < threshold) &&
	(fabs(b_diff) < threshold)) {

      label[ix][iy] = 1;
      nmean++;
      rmean += r_diff/nmean;
      gmean += g_diff/nmean;
      bmean += b_diff/nmean;

      // Add unchecked neighboring nodes to stack:
      if ((iy > 0) && (!label[ix][iy-1]))
	stack.push_back(Node(ix, iy-1));
      if ((iy < ny-1) && (!label[ix][iy+1]))
	stack.push_back(Node(ix, iy+1));
      if ((ix > 0) && (!label[ix-1][iy]))
	stack.push_back(Node(ix-1, iy));
      if ((ix < nx-1) && (!label[ix+1][iy]))
	stack.push_back(Node(ix+1, iy));

    }
    else {
      // Label pixel as checked but not selected
      label[ix][iy] = 2;
    }

  }

  // Remove checked labels
  for (int ix = 0; ix < nx; ix++) {
    for (int iy = 0; iy < ny; iy++) {
      if (label[ix][iy] > 1) label[ix][iy] = 0;
    }
  }

  lua_pushlightuserdata(L, out_ptr);
  return 1;

}

static const struct luaL_reg rgbselect_lib [] = {
  {"select", lua_select},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_rgbselect (lua_State *L) {
  luaL_register(L, "rgbselect", rgbselect_lib);

  return 1;
}

