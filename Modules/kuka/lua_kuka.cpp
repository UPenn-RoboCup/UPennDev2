#include <lua.hpp>
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"

#define YOUBOT_CONFIGURATIONS_DIR "/home/youbot/youbot_driver/config"

// Namespace
using namespace youbot;

// Access the robot modules
YouBotBase* ybBase = NULL;
YouBotManipulator* ybArm = NULL;
// Container for desired joint angles
JointAngleSetpoint* desiredJointAngle;

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

// Shutdown the modules
static int lua_shutdown_base(lua_State *L) {
  delete ybBase;
}
static int lua_shutdown_arm(lua_State *L) {
  delete ybArm;
}

// Calibrate arm
static int lua_calibrate_arm(lua_State *L) {
  if(ybArm){
    ybArm->calibrateManipulator();
  	// Return true
  	lua_pushboolean(L,1);
  	return 1;
  }
	// Return false if not available
	lua_pushboolean(L,0);
	return 1;
}

// Set base speed
static int lua_set_base_velocity(lua_State *L) {
  // Make the appropriate quantities
	quantity<si::velocity> longitudinalVelocity = lua_tonumber(L, 1) * meter_per_second;
	quantity<si::velocity> transversalVelocity = lua_tonumber(L, 2) * meter_per_second;
	quantity<si::angular_velocity> angularVelocity = lua_tonumber(L, 3) * radian_per_second;
  // Set the base
  ybBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
}


static const struct luaL_reg kuka_lib [] = {
	{"init_base", lua_init_base},
	{"init_arm", lua_init_arm},
  //
  {"shutdown_base", lua_shutdown_base},
  {"shutdown_arm", lua_shutdown_arm},
  //
  {"calibrate_arm", lua_calibrate_arm},
  //
  {"set_base_velocity", lua_set_base_velocity},
  //
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
