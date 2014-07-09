//#include "lua.h"
//#include "lualib.h"
//#include "lauxlib.h"

#include <lua.hpp>
#include <stdint.h>
#include <math.h>
#include <vector>


#ifdef TORCH
#include <torch/luaT.h>
#ifdef __cplusplus
extern "C"
{
#endif
#include <torch/TH/TH.h>
#ifdef __cplusplus
}
#endif
#endif

static const int NMAX = 320;
//static int countJ[NMAX];
static int minJ[NMAX];

static const int widthMin = 8;
static const int widthMax = 30; //TODO

// Loop through scan lines and check connected black regions
int obstacleState (bool is_lower) {
	enum {STATE_NONE, STATE_FIELD, STATE_OBS};
	static int state = STATE_NONE;
	static int width = 0;

	switch (state) {
    case STATE_NONE:
      if (!is_lower) {
        state = STATE_FIELD;
      } else {
      	state = STATE_OBS;
      }
      break;
    case STATE_FIELD:
      if (is_lower) {
      	state = STATE_OBS;
      	width = 1;
      }
      break;
    case STATE_OBS:
      if (is_lower) {
      	width++;
      } else {
      	state = STATE_FIELD;
      	return width;
      }
  }
  return 0;
}



//Do a tilted scan at labelB image
//and return the lower boundary for every scaned lines
int lua_obstacles(lua_State *L) {
	uint8_t *im_ptr;
	int m, n, max_gap;
	double tiltAngle;
	uint8_t mask;
  
  if (lua_islightuserdata(L, 1)) {
    im_ptr = (uint8_t *) lua_touserdata(L, 1);
    m = luaL_checkint(L, 2);
    n = luaL_checkint(L, 3);
    mask = luaL_optinteger(L, 4, 8); //field
    tiltAngle = luaL_optnumber(L, 5, 0.0);
    max_gap = luaL_optinteger(L, 6, 1 );
  }
  #ifdef TORCH
  else if(luaT_isudata(L,1,"torch.ByteTensor")) {
    THByteTensor* b_t =
    (THByteTensor *) luaT_checkudata(L, 1, "torch.ByteTensor");
    im_ptr = b_t->storage->data;
    n = b_t->size[0];
    m = b_t->size[1];
    mask = luaL_optinteger(L, 2, 8);
    tiltAngle = luaL_optnumber(L, 3, 0.0);
    max_gap = luaL_optinteger(L, 4, 1 );
  }
  #endif
  else {
    return luaL_error(L, "Input image invalid");
  }

  double increment= tan(tiltAngle);
  int index_offset = m/2;

  // Initialize arrays
  for (int i = 0;i<m; i++ ){
    minJ[i] = n;
  }

  // Iterate through image getting projection statistics
  //for (int i = -index_offset; i < m+index_offset; i++) {
  for (int i = 0; i < m; i++) {
    int flag = 0; //0 for initial, 1 for scanning, 2 for ended
    int gap = 0;
    for (int j = n-1; j >=0 ; j--) {
    	// if scan on this column is ended
      if (flag == 2) {
      	flag = 0;
      	break;
      }
      double shift = (double) j*increment;
      int index_i =(int) (i+shift+0.5); //round up

      //check current pixel that is in the image
      if ( (index_i>=0) && (index_i<m) ) {
        int index_ij = j*m + index_i;
        uint8_t pixel = *(im_ptr+index_ij);
        if (pixel & mask) {
          flag=1;
          if (j<minJ[index_i]) minJ[index_i]=j;
        }else{
          if (flag==1) {
            gap++;
            if (gap>max_gap) flag=2; //end scan
          }
        } // check if within mask
      } // check if in image
    } // scan j
  } // scan i
  
  // A threshold for filtering obstacles
  int sumJ = 0;
  for (int i=0; i<m; i++) {
  	sumJ += minJ[i];
  }
  //double threshold = (double) sumJ/m*1.5; //TODO
  int threshold = (int) sumJ/m*1.5; //TODO
  threshold = (threshold<n)? threshold:n;

  std::vector<int> obstacleW;
  std::vector<int> obstacleI;
  std::vector<int> obstacleJ;
  int j0 = 0;
  for (int i=0; i<m; i++) {
  	int width = obstacleState( minJ[i] >= threshold );
  	if ( (width>=widthMin) && (width<=widthMax) ) {
  	  int iObstacle = i - (width+1)/2;
  	  int jObstacle = (j0 + minJ[i])/2; // TODO: improve later
  		obstacleW.push_back(width);
  		obstacleI.push_back(iObstacle);
  		obstacleJ.push_back(jObstacle);
    } else {
    	j0 = minJ[i];
    }
  }

  //TODO: sort according to width in image

  int nObstacle = sizeof(obstacleI);
  lua_createtable(L, nObstacle, 0);
  for (int i=0; i<nObstacle; i++) {
  	lua_createtable(L, 0, 2);
  	// width field
  	lua_pushstring(L, "width");
  	lua_pushnumber(L, obstacleW[i]);
  	lua_settable(L, -3);

  	// position field
  	lua_pushstring(L, "position");
  	lua_createtable(L, 2, 0);
  	lua_pushnumber(L, obstacleI[i]);
  	lua_rawseti(L, -2, 1);
  	lua_pushnumber(L, obstacleJ[i]);
  	lua_rawseti(L, -2, 2);
  	lua_settable(L, -3);

    lua_rawseti(L, -2, i+1);
  }

  return 1;
}
