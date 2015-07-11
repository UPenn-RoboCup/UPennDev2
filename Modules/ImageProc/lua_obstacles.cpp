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
static int minJ[NMAX];
// Limit on number of obstacles we check
static const int NOBS = 30;

static int widthMin, widthMax;
static int horizon;

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
	int m, n;
  int max_gap = 1;
	double tiltAngle;
  // int mask;
  uint8_t mask_g, mask_r, mask_w;

  if (lua_islightuserdata(L, 1)) {
    im_ptr = (uint8_t *) lua_touserdata(L, 1);
    m = luaL_checkint(L, 2);
    n = luaL_checkint(L, 3);
		//
		widthMin = luaL_optinteger(L, 4, 5);
		widthMax = luaL_optinteger(L, 5, 25);
    horizon = luaL_optinteger(L, 6, 22);
    tiltAngle = luaL_optnumber(L, 7, 0.0);
    // No need to input args
    mask_g = luaL_optinteger(L, 8, 8); // field green
    mask_r = luaL_optinteger(L, 9, 1); // orange ball
    mask_w = luaL_optinteger(L, 10, 16); // line white
  }
	else if( lua_type(L, 1) == LUA_TNUMBER ){
		im_ptr = (uint8_t *)luaL_optlong(L, 1, 0);
    if (im_ptr == NULL) {
      return luaL_error(L, "Input image bad");
    }
		m = luaL_checkint(L, 2);
    n = luaL_checkint(L, 3);
		//
		widthMin = luaL_optinteger(L, 4, 5);
		widthMax = luaL_optinteger(L, 5, 25);
    horizon = luaL_optinteger(L, 6, 22);
    tiltAngle = luaL_optnumber(L, 7, 0.0);
    // No need to input args
    mask_g = luaL_optinteger(L, 8, 8); // field green
    mask_r = luaL_optinteger(L, 9, 1); // orange ball
    mask_w = luaL_optinteger(L, 10, 16); // line white
	}
  #ifdef TORCH
  else if(luaT_isudata(L,1,"torch.ByteTensor")) {
    THByteTensor* b_t =
    (THByteTensor *) luaT_checkudata(L, 1, "torch.ByteTensor");
    im_ptr = b_t->storage->data;
    n = b_t->size[0];
    m = b_t->size[1];
		widthMin = luaL_optinteger(L, 2, 5);
		widthMax = luaL_optinteger(L, 3, 25);
    horizon = luaL_optinteger(L, 4, 22);
    tiltAngle = luaL_optnumber(L, 5, 0.0);

    // No need to input args
    mask_g = luaL_optinteger(L, 6, 8); //field green
    mask_r = luaL_optinteger(L, 7, 1); //orange ball
    mask_w = luaL_optinteger(L, 8, 16); //line white
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
  for (int i = -index_offset; i < m+index_offset; i++) {
    int flag = 0; //0 for initial, 1 for scanning, 2 for ended
    int gap = 0;
    //for (int j = 0; j <n ; j++) {
    for (int j = n-1; j>=0 ; j--) {
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
        if ((pixel&mask_g) || (pixel&mask_r) || (pixel&mask_w)) {
          flag=1;
          if (j<minJ[index_i]) {
            if (j>horizon) minJ[index_i]=j;
            else minJ[index_i] = horizon;
          }
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
  int lowest = 0;
  for (int i=0; i<m; i++) {
  	sumJ += minJ[i];
  	if (minJ[i]>lowest) lowest = minJ[i];
  }
  int threshold = (int) sumJ/m*1; //TODO
  threshold = (threshold<n)? threshold:n;
	//printf("THRESHOLD: %d\n", threshold);

  int obstacleW[NOBS];
  int obstacleI[NOBS];
  int obstacleJ[NOBS];

  static int j0 = 0;
  int obs_count = 0;
  for (int i=0; i<m; i++) {
  	int obs_width = obstacleState( minJ[i] >= threshold );
  	//printf("obs_width:%d\n", obs_width);
  	if ( (obs_width>=widthMin) && (obs_width<=widthMax) ) {
			//printf("width? %d widthMin: %d, widthMax: %d\n", obs_width, widthMin, widthMax);
  	  int iObstacle = i - (obs_width+1)/2;
  	  //int jObstacle = (j0 + minJ[i])/2; // TODO: improve later
  	  int jObstacle = minJ[iObstacle];
  		obstacleW[obs_count] = obs_width;
  		obstacleI[obs_count] = iObstacle;
  		obstacleJ[obs_count] = jObstacle;
      obs_count = obs_count + 1;
      if (obs_count==NOBS) break;
    } else {
      j0 = minJ[i];
    }
  }

  lua_createtable(L, obs_count, 0);
  for (int i=0; i<obs_count; i++) {
  	lua_createtable(L, 0, 2);
  	// width field
  	//printf("obs count:%d, OBS WIDTH: %d\n", obs_count, obstacleW[i]);
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
