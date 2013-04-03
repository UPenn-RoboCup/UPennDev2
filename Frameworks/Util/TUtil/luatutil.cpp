/**
 * Lua module to expose some common c utilties
 *
 * University of Pennsylvania
 * 2010
 */
#include "luatutil.h"

int lua_tensor2qimage(lua_State *L) {
	THByteTensor * mapline_t = (THByteTensor *) luaT_checkudata(L, 1, "torch.ByteTensor");
  // Format_RGB32: The image is stored using a 32-bit RGB format (0xffRRGGBB).
	uint32_t * scanline = (uint32_t *)lua_touserdata(L, 2); 
  if ((scanline == NULL) || !lua_islightuserdata(L, 2)) {
    return luaL_error(L, "Input scanline not light user data");
  }
	const int sizex = mapline_t->size[0];
	for(int i=0;i<sizex;i++)
		scanline[i] = THTensor_fastGet1d(mapline_t,i)<<8;
	return 0;
}

int lua_get_pointer(lua_State *L) {
	THFloatTensor * data_t = (THFloatTensor *) luaT_checkudata(L, 1, "torch.FloatTensor");
	THFloatStorage * data_s = (THFloatStorage *)(data_t->storage);
	lua_pushlightuserdata(L, (float*)(data_s->data));
	return 1;
}

static const struct luaL_reg tutil_lib [] = {
  {"tensor2qimage", lua_tensor2qimage},
	{"get_pointer", lua_get_pointer},

  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_tutil (lua_State *L) {
  luaL_register(L, "tutil", tutil_lib);
  return 1;
}
