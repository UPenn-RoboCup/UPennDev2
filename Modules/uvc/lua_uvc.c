/*
  Author: Daniel D. Lee <ddlee@seas.upenn.edu>, 05/10
  	    : Stephen McGill 10/10
        : Yida Zhang 05/13
*/

#ifdef _cplusplus
extern "C" {
#endif
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
#ifdef _cplusplus
}
#endif

#include "v4l2.h"
#include "timeScalar.h"

#include <stdint.h>
#include <stdio.h>

/* metatable name for uvc */
#define MT_NAME "uvc_mt"

/* default video device if not named */
#define VIDEO_DEVICE "/dev/video0"

/* default image size */
#define WIDTH 640
#define HEIGHT 480


static uint8_t* yuyv_rotate(uint8_t* frame, int width, int height) {
  int i;
  //SJ: I maintain a second buffer here
  //So that we do not directly rewrite on camera buffer address

  static uint8_t frame2[160*120*4];

  //printf("WIDTH HEIGHT:%d %d\n",width,height);

  int siz = width*height/2;
  for (int i=0;i<siz/2;i++){
    int index_1 = i*4;
    int index_2 = (siz-1-i)*4;
    uint8_t x1,x2,x3,x4;
    frame2[index_2] = frame[index_1+2];
    frame2[index_2+1] = frame[index_1+1];
    frame2[index_2+2] = frame[index_1];
    frame2[index_2+3] = frame[index_1+3];

    frame2[index_1]=frame[index_2+2];
    frame2[index_1+1]=frame[index_2+1];
    frame2[index_1+2]=frame[index_2];
    frame2[index_1+3]=frame[index_2+3];

  }
  return frame2;


/*
  uint32_t* frame32 = (uint32_t*)frame;

  // Swap top and bottom
  for(i=0;i<height/2;i++){

    uint32_t* row1addr = frame32+i*width/2;
    uint32_t* row2addr = frame32+(height-1-i)*width/2;
    row_swap( row1addr, row2addr, width );
  
    // Swap the yuyv byte order for the swapped rows
    int j;
    for(j=0;j<width/4;j++){// width/4 since frame32 is width/2 long
      yuyv_px_swap( (uint8_t*)(row1addr+j), (uint8_t*)(row1addr+width/2-1-j) );
    }
    for(j=0;j<width/4;j++){// width/4 since frame32 is width/2 long
      yuyv_px_swap( (uint8_t*)(row2addr+j), (uint8_t*)(row2addr+width/2-1-j) );
    }

  }
*/

}

static v4l2_device * lua_checkuvc(lua_State *L, int narg) {
    void *ud = luaL_checkudata(L, narg, MT_NAME);
    luaL_argcheck(L, ud != NULL, narg, "invalid uvc userdata");
    return (v4l2_device *) ud;
}

static int lua_uvc_index(lua_State *L) {
    if (!lua_getmetatable(L, 1)) { /* push metatable */
        lua_pop(L, 1); 
        return 0;
    }
    lua_pushvalue(L, 2); /* copy key */
    lua_rawget(L, -2); /* get metatable function */
    lua_remove(L, -2);  /* delte metatable */
    return 1;
}

static int lua_uvc_delete(lua_State *L) {
    v4l2_device *ud = lua_checkuvc(L, 1);
    if (ud->init)
        v4l2_stream_off(ud);
        if (v4l2_close(ud) != 0)
            luaL_error(L, "Closing video device Error");
    return 0;
}

static int lua_uvc_init(lua_State *L) {
    const char * video_device = luaL_optstring(L, 1, VIDEO_DEVICE);
    v4l2_device *ud = (v4l2_device *)lua_newuserdata(L, sizeof(v4l2_device));
    ud->width = luaL_optint(L, 2, WIDTH);
    ud->height = luaL_optint(L, 3, HEIGHT);
    ud->pixelformat = luaL_optstring(L, 4, "yuyv");
    /* default 30 fps */
    ud->fps_num = luaL_optint(L, 5, 1);
    ud->fps_denum = luaL_optint(L, 6, 30);

    ud->init = 0;
    ud->count = 0;
    ud->ctrl_map = NULL;
    ud->menu_map = NULL;
    ud->buf_len = NULL;
    ud->buffer = NULL;

    ud->fd = v4l2_open(video_device);

    if (ud->fd > 0){
        ud->init = 1;
        v4l2_init(ud);
        v4l2_stream_on(ud);
    } else
        luaL_error(L, "Could not open video device");
/*
    fprintf(stdout, "open video device %d\n", ud->fd);
    fflush(stdout);
*/

    luaL_getmetatable(L, MT_NAME);
    lua_setmetatable(L, -2);
    return 1;
}

static int lua_uvc_fd(lua_State *L) {
    v4l2_device *ud = lua_checkuvc(L, 1);
    if (ud->init)
        lua_pushinteger(L, ud->fd);
    else
        luaL_error(L, "uvc not init");
    return 1;
}

static int lua_uvc_get_width(lua_State *L) {
    v4l2_device *ud = lua_checkuvc(L, 1);
    if (ud->init)
        lua_pushinteger(L, ud->width);
    else
        luaL_error(L, "uvc not init");
    return 1;
}

static int lua_uvc_get_height(lua_State *L) {
    v4l2_device *ud = lua_checkuvc(L, 1);
    if (ud->init)
        lua_pushinteger(L, ud->height);
    else
        luaL_error(L, "uvc not init");
    return 1;
}

static int lua_uvc_set_param(lua_State *L) {
    v4l2_device *ud = lua_checkuvc(L, 1);
    const char *param = luaL_checkstring(L, 2);
    int value = luaL_checkint(L, 3);
 
    int ret = v4l2_set_ctrl(ud, param, value);
    lua_pushnumber(L, ret);
    return 1;
}

static int lua_uvc_get_param(lua_State *L) {
    v4l2_device *ud = lua_checkuvc(L, 1);
    const char *param = luaL_checkstring(L, 2);
    int value;
    v4l2_get_ctrl(ud, param, &value);
    lua_pushnumber(L, value);
    return 1;
}

static int lua_uvc_get_raw(lua_State *L) {
    v4l2_device *ud = lua_checkuvc(L, 1);
    int buf_num = v4l2_read_frame(ud);
    if( buf_num < 0 ){
      lua_pushnumber(L,buf_num);
      return 1;
    }
    uint32_t* image = (uint32_t*)ud->buffer[buf_num];
    ud->count ++;
    lua_pushlightuserdata(L, image);
    lua_pushnumber(L, ud->buf_len[buf_num]);
    lua_pushnumber(L, ud->count);
    lua_pushnumber(L, time_scalar());
    return 4;
}

static int lua_uvc_get_rotated(lua_State *L) {
    v4l2_device *ud = lua_checkuvc(L, 1);
    int buf_num = v4l2_read_frame(ud);
    if( buf_num < 0 ){
      lua_pushnumber(L,buf_num);
      return 1;
    }
    uint32_t* image = (uint32_t*)ud->buffer[buf_num];
    uitn8_t* rot_image = yuyv_rotate( (uint8_t*)image, width, height );
    ud->count ++;
    lua_pushlightuserdata(L, rot_image);
    lua_pushnumber(L, ud->buf_len[buf_num]);
    lua_pushnumber(L, ud->count);
    lua_pushnumber(L, time_scalar());
    return 4;
}

static int lua_uvc_reset_resolution(lua_State *L) {
    v4l2_device *ud = lua_checkuvc(L, 1);
    ud->width = luaL_checkint(L, 2);
    ud->height = luaL_checkint(L, 3);
    ud->pixelformat = luaL_optstring(L, 4, "yuyv");

    v4l2_stream_off(ud);
    v4l2_uninit_mmap(ud);
    v4l2_close_query(ud);

    v4l2_init(ud);
    v4l2_stream_on(ud);
    return 1;
}

static const struct luaL_Reg uvc_functions [] = {
    {"init", lua_uvc_init},
    {NULL, NULL}
};

static const struct luaL_Reg uvc_methods [] = {
    {"descriptor", lua_uvc_fd},
    {"close", lua_uvc_delete},
    {"get_width", lua_uvc_get_width},
    {"get_height", lua_uvc_get_height},
    {"reset", lua_uvc_reset_resolution},
    {"set_param", lua_uvc_set_param},
    {"get_param", lua_uvc_get_param},
    {"get_image", lua_uvc_get_raw},
    {"get_rotated_image", lua_uvc_get_rotated},
    {"__index", lua_uvc_index},
    {"__gc", lua_uvc_delete},
    {NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_uvc (lua_State *L) {
    /* create metatable for uvc module */
    luaL_newmetatable(L, MT_NAME);

#if LUA_VERSION_NUM == 502
    luaL_setfuncs(L, uvc_methods, 0);
	  luaL_newlib(L, uvc_functions);
#else 
    luaL_register(L, NULL, uvc_methods);
    luaL_register(L, "uvc", uvc_functions);
#endif
    return 1;
}
