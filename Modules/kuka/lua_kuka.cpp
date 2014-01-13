#include <lua.hpp>
#include "youbot/YouBotBase.hpp"
#include "youbot/YouBotManipulator.hpp"

#define YOUBOT_CONFIGURATIONS_DIR "/home/youbot/youbot_driver/config"

// Namespace
using namespace youbot;

// Access the robot modules
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

// Shutdown the modules
static int lua_shutdown_base(lua_State *L) {
  delete ybBase;
}
static int lua_shutdown_arm(lua_State *L) {
  delete ybArm;
}

// Calibrate arm
static int lua_calibrate_arm(lua_State *L) {  
  
  if(!ybArm){
    return luaL_error(L,"Arm is not initialized!");
  }
  
  ybArm->calibrateManipulator();

}

// Set base speed
static int lua_set_base_velocity(lua_State *L) {
  
  static quantity<si::velocity> longitudinalVelocity;
  static quantity<si::velocity> transversalVelocity;
  static quantity<si::angular_velocity> angularVelocity;
  
  double dx = (double)lua_tonumber(L, 1);
  double dy = (double)lua_tonumber(L, 2);
  double da = (double)lua_tonumber(L, 3);

  /*
  printf("dx: %lf, dy: %lf, da: %lf \n",dx,dy,da);
  fflush(stdout);
  */
  
  // Make the appropriate quantities
  longitudinalVelocity = dx * meter_per_second;
  transversalVelocity  = dy * meter_per_second;
  angularVelocity      = da * radian_per_second;
  
  // Set the base
  ybBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
  
  return 0;
}

// Set arm angles
static int lua_set_arm_angle(lua_State *L) {
  static JointAngleSetpoint desiredJointAngle;
  if(!ybArm){return luaL_error(L,"Arm is not initialized!");}
  
  int joint_id = luaL_checkint(L, 1);
  double joint_angle = (double)lua_tonumber(L, 2);
  /*
  printf("joint_id: %d, angle: %lf\n", joint_id, joint_angle);
  fflush(stdout);
  */
  
  // Convert the format
  desiredJointAngle.angle = joint_angle * radian;
  // Send the angle to the robot
  ybArm->getArmJoint(joint_id).setData(desiredJointAngle);
  return 0;
}

static int lua_set_arm_max_positioning_speed(lua_State *L) {
  static MaximumPositioningSpeed maxPositioningSpeed;
  
  int joint_id = luaL_checkint(L, 1);
  double max_speed = (double)lua_tonumber(L, 2);
  
  maxPositioningSpeed.setParameter(max_speed * radian_per_second);
  ybArm->getArmJoint(joint_id).setConfigurationParameter(maxPositioningSpeed);  
  
  return 0;
}

// Get data about the arm
static int lua_get_arm_position(lua_State *L) {
  static JointSensedAngle sensed;
  int joint_id = luaL_checkint(L, 1);
  ybArm->getArmJoint(joint_id).getData(sensed);
  lua_pushnumber(L, sensed.angle.value() );
  return 1;
}
static int lua_get_arm_velocity(lua_State *L) {
  static JointSensedVelocity sensed;
  int joint_id = luaL_checkint(L, 1);
  ybArm->getArmJoint(joint_id).getData(sensed);
  lua_pushnumber(L, sensed.angularVelocity.value() );
  return 1;
}
static int lua_get_arm_torque(lua_State *L) {
  static JointSensedTorque sensed;
  int joint_id = luaL_checkint(L, 1);
  ybArm->getArmJoint(joint_id).getData(sensed);
  lua_pushnumber(L, sensed.torque.value() );
  return 1;
}
//
static int lua_get_arm_encoder(lua_State *L) {
  static JointSensedEncoderTicks sensed;
  int joint_id = luaL_checkint(L, 1);
  ybArm->getArmJoint(joint_id).getData(sensed);
  lua_pushnumber(L, sensed.encoderTicks );
  return 1;
}
static int lua_get_arm_rpm(lua_State *L) {
  static JointSensedRoundsPerMinute sensed;
  int joint_id = luaL_checkint(L, 1);
  ybArm->getArmJoint(joint_id).getData(sensed);
  lua_pushnumber(L, sensed.rpm );
  return 1;
}
static int lua_get_arm_current(lua_State *L) {
  static JointSensedCurrent sensed;
  int joint_id = luaL_checkint(L, 1);
  ybArm->getArmJoint(joint_id).getData(sensed);
  lua_pushnumber(L, sensed.current.value() );
  return 1;
}
static int lua_get_arm_pwm(lua_State *L) {
  static JointSensedPWM sensed;
  int joint_id = luaL_checkint(L, 1);
  ybArm->getArmJoint(joint_id).getData(sensed);
  lua_pushnumber(L, sensed.pwm );
  return 1;
}

static const struct luaL_reg kuka_lib [] = {
	{"init_base", lua_init_base},
	{"init_arm", lua_init_arm},
  //
  {"shutdown_base", lua_shutdown_base},
  {"shutdown_arm", lua_shutdown_arm},
  //
  {"set_base_velocity", lua_set_base_velocity},
  //
  {"calibrate_arm", lua_calibrate_arm},
  {"set_arm_max_positioning_speed", lua_set_arm_max_positioning_speed},
  {"set_arm_angle", lua_set_arm_angle},
  //
  {"get_arm_position", lua_get_arm_position},
  {"get_arm_velocity", lua_get_arm_velocity},
  {"get_arm_torque", lua_get_arm_torque},
  //
  {"get_arm_encoder", lua_get_arm_encoder},
  {"get_arm_rpm", lua_get_arm_rpm},
  {"get_arm_current", lua_get_arm_current},
  {"get_arm_pwm", lua_get_arm_pwm},
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
