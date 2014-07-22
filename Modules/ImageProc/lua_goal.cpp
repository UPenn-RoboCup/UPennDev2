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
// Limit on number of goal we check
static const int NPOSTS = 10;

//TODO: put into Config
static int widthMin, widthMax;

// Loop through scan lines and check connected regions
int connectRegion (bool is_lower) {
	enum {STATE_NONE, STATE_FIELD, STATE_POST};
	static int state = STATE_NONE;
	static int width = 0;

	switch (state) {
    case STATE_NONE:
      if (!is_lower) {
        state = STATE_FIELD;
      } else {
      	state = STATE_POST;
      }
      break;
    case STATE_FIELD:
      if (is_lower) {
      	state = STATE_POST;
      	width = 1;
      }
      break;
    case STATE_POST:
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
int lua_goal(lua_State *L) {
	uint8_t *im_ptr;
	int m, n, max_gap;
	double tiltAngle;
  uint8_t mask_y;
  
  if (lua_islightuserdata(L, 1)) {
    im_ptr = (uint8_t *) lua_touserdata(L, 1);
    m = luaL_checkint(L, 2);
    n = luaL_checkint(L, 3);
    mask_y = luaL_optinteger(L, 4, 8);
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
		widthMin = luaL_optinteger(L, 2, 5);
		widthMax = luaL_optinteger(L, 3, 25);
    tiltAngle = luaL_optnumber(L, 4, 0.0);
    
    mask_y = luaL_optinteger(L, 5, 2); //yellow
    max_gap = luaL_optinteger(L, 6, 1);
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
        if (!(pixel&mask_y)) {  // not yellow
          flag=1;
          if (j<minJ[index_i]) minJ[index_i]=j;
        }else{
          if (flag==1) {
            // if (gap>max_gap) flag=2; //end scan
            flag = 2;
          }
        } // check if within mask
      } // check if in image
    } // scan j
  } // scan i
  
  // A threshold for filtering goal
  int sumJ = 0;
  int lowest = 0;
  for (int i=0; i<m; i++) {
  	sumJ += minJ[i];
  	if (minJ[i]>lowest) lowest = minJ[i];
  }
  int threshold = (int) sumJ/m*1.2; //TODO
  threshold = (threshold<n)? threshold:n;
	//printf("THRESHOLD: %d\n", threshold);
	
  int postW[NPOSTS];
  int postI[NPOSTS];
  int postJ[NPOSTS];

  static int j0 = 0;
  int post_count = 0;
  for (int i=0; i<m; i++) {
  	int post_width = connectRegion( minJ[i] >= threshold );
  	//printf("post_width:%d\n", post_width);
  	if ( (post_width>=widthMin) && (post_width<=widthMax) ) {
			//printf("width? %d widthMin: %d, widthMax: %d\n", post_width, widthMin, widthMax);
  	  int iPost = i - (post_width+1)/2;
  	  int jPost = minJ[iPost];
  		postW[post_count] = post_width;
  		postI[post_count] = iPost;
  		postJ[post_count] = jPost;
      post_count = post_count + 1;
      if (post_count==NPOSTS) break;
    } else {
      j0 = minJ[i];	
    }
  }

  lua_createtable(L, post_count, 0);
  for (int i=0; i<post_count; i++) {
  	lua_createtable(L, 0, 2);
  	// width field
  	//printf("obs count:%d, OBS WIDTH: %d\n", post_count, postW[i]);
  	lua_pushstring(L, "width");
  	lua_pushnumber(L, postW[i]);
  	lua_settable(L, -3);

  	// position field
  	lua_pushstring(L, "position");
  	lua_createtable(L, 2, 0);
  	lua_pushnumber(L, postI[i]);
  	lua_rawseti(L, -2, 1);
  	lua_pushnumber(L, postJ[i]);
  	lua_rawseti(L, -2, 2);
  	lua_settable(L, -3);

    lua_rawseti(L, -2, i+1);
  }

  return 1;
}
