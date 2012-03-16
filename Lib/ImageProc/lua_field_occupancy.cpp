#ifdef __cplusplus
extern "C" {
#endif

#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"

#ifdef __cplusplus
}
#endif

#include <stdint.h>
#include <math.h>

typedef unsigned char uint8;

static uint8 colorBall = 0x01;
static uint8 colorField = 0x08;
static uint8 colorWhite = 0x10;

inline bool isFree(uint8 label) 
{
  return (label & colorField) || (label & colorBall) || (label & colorWhite);
}

int lua_field_occupancy(lua_State *L) {
  // Check arguments
  uint8_t *im_ptr = (uint8_t *) lua_touserdata(L, 1);
  if ((im_ptr == NULL) || !lua_islightuserdata(L, 1)) {
    return luaL_error(L, "Input image not light user data");
  }  
  int ni = luaL_checkint(L, 2);
  int nj = luaL_checkint(L, 3);
  const int nRegions = ni;

  int countup[nRegions];
  int countdown[nRegions];
  int count[nRegions];
  int flag[nRegions] ;

  for (int i = 0; i < nRegions; i++) {
    count[i] = 0;
    flag[i] = 0;
    countup[i] = 0;
    countdown[i] = 0;
  }

  // Scan vertical lines: Uphalf
  for (int i = 0; i < ni; i++) {
    int iRegion = nRegions*i/ni;
    uint8 *im_row = im_ptr + i;
    for (int j = 0; j < nj/2; j++) {
      uint8 label = *im_row;
      if (isFree(label)) {
        countup[iRegion]++;
      }
      im_row += ni;
    }
  }
  uint8 *im_ptrdown = im_ptr + (int)(ni * round(nj/2));
  // Scan vertical lines: downhalf
  for (int i = 0; i < ni; i++) {
    int iRegion = nRegions*i/ni;
    uint8 *im_row = im_ptrdown + i;
    for (int j = nj/2; j < nj; j++) {
      uint8 label = *im_row;
      if (isFree(label)) {
        countdown[iRegion]++;
      }
      im_row += ni;
    }
  }
  
  // Evaluate bound
  for (int i = 0; i < nRegions; i++){
    count[i] = countup[i] + countdown[i];
    // whole free, flag <- 2;
    if (countup[i] == nj/2){
      count[i] = nj;
      flag[i] = 2;
      continue;
    }
    // whole block, flag <- 3
    if (count[i] == 0){
      flag[i] = 3;
      continue;
    }
    int pxIdx = (nj - count[i] + 1) * ni + i;
    uint8 label = *(im_ptr + pxIdx);
    if (isFree(label)) 
      flag[i] = 1;
    else {
      //printf("Seeking\n");
      int j = nj - count[i] + 1;
      for (; j < nj; j++){
        int searchIdx = j * ni + i;
        uint8 searchLabel = *(im_ptr + searchIdx);
        if (isFree(searchLabel)) 
            break;
      }
      count[i] = nj - j + 1;
      flag[i] = 1;
    }
  }
  
  // return state
  lua_createtable(L,0,2);
  
  lua_pushstring(L,"range");
  lua_createtable(L,nRegions,0);
  for (int i = 0; i < nRegions; i++){
    lua_pushinteger(L, count[i]);
    lua_rawseti(L, -2, i+1);
  }
  lua_settable(L, -3);
  
  lua_pushstring(L,"flag");
  lua_createtable(L,nRegions,0);
  for (int i = 0; i < nRegions; i++){
    lua_pushinteger(L, flag[i]);
    lua_rawseti(L, -2, i+1);
  }
  lua_settable(L, -3);
  
  return 1;
}
