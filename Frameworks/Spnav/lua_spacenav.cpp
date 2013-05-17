#include <stdlib.h>
#include <lua.hpp>
#include "spnav.h"

spnav_event *sev = NULL;

static int lua_spacenav_shutdown(lua_State *L) {
	spnav_close();
	exit(0);
	return 0;
}

static int lua_spacenav_open(lua_State* L) {
	if (sev != NULL)
		spnav_close();

	// Initiate the event
	sev = new spnav_event;

	if(spnav_open()==-1)
		return luaL_error(L, "Failed to connect to the space navigator daemon.");

	return 1;
}

static int lua_spacenav_get(lua_State *L) {
	if (sev == NULL) 
		return luaL_error(L, "SpaceNav not initiate");

	spnav_poll_event(sev);
	//fprintf(stdout,"Got type: %d\n",sev->type);
	if(sev->type == SPNAV_EVENT_MOTION) {
		//printf("got motion event: t(%d, %d, %d) ", sev->motion.x, sev->motion.y, sev->motion.z);
		//printf("r(%d, %d, %d)\n", sev->motion.rx, sev->motion.ry, sev->motion.rz);
		//fflush(stdout);

		lua_createtable(L, 0, 1);

		lua_pushstring(L, "event");
		lua_pushstring(L, "motion");
		lua_settable(L, -3);

		lua_pushstring(L, "x");
		lua_pushinteger(L, sev->motion.x);
		lua_settable(L, -3);

		lua_pushstring(L, "y");
		lua_pushinteger(L, sev->motion.y);
		lua_settable(L, -3); 

		lua_pushstring(L, "z");
		lua_pushinteger(L, sev->motion.z);
		lua_settable(L, -3); 

		lua_pushstring(L, "rx");
		lua_pushinteger(L, sev->motion.rx);
		lua_settable(L, -3);

		lua_pushstring(L, "ry");
		lua_pushinteger(L, sev->motion.ry);
		lua_settable(L, -3); 

		lua_pushstring(L, "rz");
		lua_pushinteger(L, sev->motion.rz);
		lua_settable(L, -3); 
		return 1;


	} else if (sev->type == SPNAV_EVENT_BUTTON) {	/* SPNAV_EVENT_BUTTON */
		//printf("got button %s event b(%d)\n", sev->button.press ? "press" : "release", sev->button.bnum);

		lua_createtable(L, 0, 4);

		lua_pushstring(L, "event");
		lua_pushstring(L, "button");
		lua_settable(L, -3);

		lua_pushstring(L, "bpress");
		lua_pushinteger(L, sev->button.press);
		lua_settable(L, -3); 

		lua_pushstring(L, "bnum");
		lua_pushinteger(L, sev->button.bnum);
		lua_settable(L, -3); 
		return 1;
	}

	return 0;
}

static const struct luaL_Reg spacenav_lib [] = {
	{"open", lua_spacenav_open},
	{"get", lua_spacenav_get},
	{"shutdown", lua_spacenav_shutdown}, 
	{NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_spacenav(lua_State *L) {
#if LUA_VERSION_NUM == 502
	luaL_newlib( L, spacenav_lib );
#else
	luaL_register(L, "spacenav", spacenav_lib);
#endif
	return 1;
}
