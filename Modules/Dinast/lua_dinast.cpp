
#include <lua.hpp>
#include <stdlib.h>
#include "dinast_grabber.h"

const char* dinast_mt = "DINAST_MT";

typedef struct {
  libusb_context *ctx;
  struct libusb_device_handle *device;
  unsigned char *img1;
  unsigned char *img2;
} structDinast;

static structDinast * lua_checkdinast(lua_State *L, int narg) {
  void *ud = luaL_checkudata(L, narg, dinast_mt);
  luaL_argcheck(L, ud != NULL, narg, "invalid dinast device");
  return (structDinast *)ud;
}

static int lua_dinast_index(lua_State *L) {
  structDinast *p = lua_checkdinast(L, 1);

  // Get index through metatable:
  if (!lua_getmetatable(L, 1)) {lua_pop(L, 1); return 0;} // push metatable
  lua_pushvalue(L, 2); // copy key
  lua_rawget(L, -2); // get metatable function
  lua_remove(L, -2); // delete metatable
  return 1;
}

static int lua_dinast_delete(lua_State *L) {
  structDinast *p = lua_checkdinast(L, 1);
  if (p->device != NULL) {
    stop(p->device);
    closeDevice(p->device);
  }
  if (p->ctx != NULL) libusb_exit(p->ctx);
  if (p->img1 != NULL)
    free(p->img1);
  if (p->img2 != NULL)
    free(p->img2);

  return 1;
}

static int lua_dinast_init(lua_State *L) {
  structDinast *ud = (structDinast *)lua_newuserdata(L, sizeof(structDinast));

  ud->device = findDevice(ud->ctx);
  if (ud->device == NULL) {
    luaL_error(L, "Couldn't find any Dinast devices attached!");
  } else {
    printf("Found Dinast device %s\n", getDeviceVersion(ud->device).c_str());
    fflush(stdout);
  }

  if (!start(ud->device)) {
    luaL_error(L, "Could not start device!");
  }

  ud->img1 = (unsigned char*) malloc(IMAGE_SIZE);
  ud->img2 = (unsigned char*) malloc(IMAGE_SIZE);

  luaL_getmetatable(L, dinast_mt);
  lua_setmetatable(L, -2);
  return 1;
}

static int lua_dinast_grab(lua_State *L) {
  structDinast *ud = lua_checkdinast(L, 1);
  
  if (readImage(ud->device, ud->img1, ud->img2)) {
#ifdef DEBUG
    fprintf(stdout, "Got a reading!\n");
    fflush(stdout);
#endif
    lua_pushlightuserdata(L, ud->img1);
    lua_pushlightuserdata(L, ud->img2);
  } else {
    lua_pushnumber(L, 0);
    lua_pushnumber(L, 0);
  }
  return 2;
}

static const luaL_Reg dinast_functions[] = {
  {"init", lua_dinast_init},
  {NULL, NULL}
};

static const luaL_Reg dinast_methods[] = {
  {"get_image", lua_dinast_grab},
  {"__gc", lua_dinast_delete},
  {"__index", lua_dinast_index},
  {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_dinast(lua_State *L) {
  luaL_newmetatable(L, dinast_mt);

#if LUA_VERSION_NUM == 502
  luaL_setfuncs(L, dinast_methods, 0);
  luaL_newlib(L, dinast_functions);
#else
  luaL_register(L, NULL, dinast_methods);
  luaL_register(L, "dinast", dinast_functions);
#endif
  return 1;
}
