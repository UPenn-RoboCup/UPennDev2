#include <lua.hpp>
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"

#define YOUBOT_CONFIGURATIONS_DIR "/home/youbot/youbot_driver/config"

// Namespace
using namespace youbot;

// Important globals
YouBotBase* ybBase = NULL;
YouBotManipulator* ybArm = NULL;

// Initialize the wheeled base module
static int lua_init_base(lua_State *L) {
	try {
		ybBase = new YouBotBase("youbot-base", YOUBOT_CONFIGURATIONS_DIR);
		// Why the commutation?
		ybBase->doJointCommutation();
	} catch (std::exception& e) {
		return luaL_error(L, e.what() );
	}

	// Return true
	lua_pushboolean(L,1);
	return 1;
}

// Initialize the arm module
static int lua_init_arm(lua_State *L) {
	try {
		ybArm = new YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
		ybArm->doJointCommutation();
		//ybArm->calibrateManipulator();
	} catch (std::exception& e) {
		return luaL_error(L, e.what() );
	}

	// Return true
	lua_pushboolean(L,1);
	return 1;
}

static const struct luaL_reg kuka_lib [] = {
	{"init_base", lua_init_base},
	{"init_arm", lua_init_arm},
	{NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_kuka (lua_State *L) {
#if LUA_VERSION_NUM == 502
	luaL_newlib(L, kuka_lib);
#else
	luaL_register(L, "kuka", kuka_lib);
#endif
	return 1;
}
