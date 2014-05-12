#include <lua.hpp>
#include "youbot_driver/youbot/YouBotBase.hpp"
#include "youbot_driver/youbot/YouBotManipulator.hpp"

#define YOUBOT_CONFIGURATIONS_DIR "/usr/local/share/config"

// Namespace
using namespace youbot;

// Access the robot modules
// NOTE: not thread safe if twice loaded
static YouBotBase* ybBase = NULL;
static YouBotManipulator* ybArm = NULL;

// Initialize the modules
static int lua_init_base(lua_State *L) {
	if(ybBase) return luaL_error(L,"Base is initialized already!");
	try {
		ybBase = new YouBotBase(
				"youbot-base", YOUBOT_CONFIGURATIONS_DIR);
		ybBase->doJointCommutation();
	} catch (std::exception& e) {
		return luaL_error(L, e.what() );
	}
	return 0;
}

static int lua_init_arm(lua_State *L) {
	if(ybArm) return luaL_error(L,"Arm is initialized already!");
	try {
		ybArm = new YouBotManipulator(
				"youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR);
		ybArm->doJointCommutation();
		ybArm->calibrateManipulator();
	} catch (std::exception& e) {
		return luaL_error(L, e.what() );
	}
	return 0;
}

// Shutdown the modules
static int lua_shutdown_base(lua_State *L) {
	if(!ybBase) return luaL_error(L,"Base is not initialized!");
	delete ybBase;
	return 0;
}
static int lua_shutdown_arm(lua_State *L) {
	if(!ybArm) return luaL_error(L,"Arm is not initialized!");
	delete ybArm;
	return 0;
}

static int lua_calibrate_gripper(lua_State *L) {
	if(!ybArm) return luaL_error(L,"Arm is not initialized!");
	ybArm->calibrateGripper();
	return 0;
}

// Set base speed
static int lua_set_base_velocity(lua_State *L) {

	static quantity<si::velocity> longitudinalVelocity;
	static quantity<si::velocity> transversalVelocity;
	static quantity<si::angular_velocity> angularVelocity;

	if(!ybBase) return luaL_error(L,"Base is not initialized!");

	double dx = (double)lua_tonumber(L, 1);
	double dy = (double)lua_tonumber(L, 2);
	double da = (double)lua_tonumber(L, 3);

	// Make the appropriate quantities
	longitudinalVelocity = dx * meter_per_second;
	transversalVelocity  = dy * meter_per_second;
	angularVelocity      = da * radian_per_second;

	// Set the base
	ybBase->setBaseVelocity(
			longitudinalVelocity, transversalVelocity, angularVelocity);

	return 0;
}

// Get base position, from encoders, and use YouBot API calculation
static int lua_get_base_position(lua_State *L) {
	static quantity<si::length> actualLongitudinalPose = 0 * meter;
	static quantity<si::length> actualTransversalPose = 0 * meter;
	static quantity<si::plane_angle> actualAngle = 0 * radian;

	if(!ybBase){return luaL_error(L,"Base is not initialized!");}

	ybBase->getBasePosition(
			actualLongitudinalPose, actualTransversalPose, actualAngle);

	// Push the numbers onto the stack
	lua_pushnumber(L, actualLongitudinalPose.value() ); //x
	lua_pushnumber(L, actualTransversalPose.value() ); //y
	lua_pushnumber(L, actualAngle.value() ); //a
	return 3;
}

static int lua_set_base_wheel(lua_State *L) {
	static JointVelocitySetpoint setVel;
	if(!ybBase) return luaL_error(L,"Base is not initialized!");
	int wheel_id = luaL_checkint(L, 1);
	double velocity = (double)lua_tonumber(L, 2);
	setVel.angularVelocity = velocity*radian_per_second;
	ybBase->getBaseJoint(wheel_id).setData(setVel);
	return 0;
}

// Set arm angles
static int lua_set_arm_angle(lua_State *L) {
	static JointAngleSetpoint desiredJointAngle;
	if(!ybArm){return luaL_error(L,"Arm is not initialized!");}

	int joint_id = luaL_checkint(L, 1);
	double joint_angle = (double)lua_tonumber(L, 2);

	// Convert the format
	desiredJointAngle.angle = joint_angle * radian;
	// Send the angle to the robot
	ybArm->getArmJoint(joint_id).setData(desiredJointAngle);
	return 0;
}

// Get data about the arm
static int lua_get_arm_position(lua_State *L) {
	static JointSensedAngle sensed;
	if(!ybArm){return luaL_error(L,"Arm is not initialized!");}
	int joint_id = luaL_checkint(L, 1);
	ybArm->getArmJoint(joint_id).getData(sensed);
	lua_pushnumber(L, sensed.angle.value() );
	return 1;
}
static int lua_get_arm_velocity(lua_State *L) {
	static JointSensedVelocity sensed;
	if(!ybArm){return luaL_error(L,"Arm is not initialized!");}
	int joint_id = luaL_checkint(L, 1);
	ybArm->getArmJoint(joint_id).getData(sensed);
	lua_pushnumber(L, sensed.angularVelocity.value() );
	return 1;
}
static int lua_get_arm_torque(lua_State *L) {
	static JointSensedTorque sensed;
	if(!ybArm){return luaL_error(L,"Arm is not initialized!");}
	int joint_id = luaL_checkint(L, 1);
	ybArm->getArmJoint(joint_id).getData(sensed);
	lua_pushnumber(L, sensed.torque.value() );
	return 1;
}
//
static int lua_get_arm_encoder(lua_State *L) {
	static JointSensedEncoderTicks sensed;
	if(!ybArm){return luaL_error(L,"Arm is not initialized!");}
	int joint_id = luaL_checkint(L, 1);
	ybArm->getArmJoint(joint_id).getData(sensed);
	lua_pushnumber(L, sensed.encoderTicks );
	return 1;
}
static int lua_get_arm_rpm(lua_State *L) {
	static JointSensedRoundsPerMinute sensed;
	if(!ybArm){return luaL_error(L,"Arm is not initialized!");}
	int joint_id = luaL_checkint(L, 1);
	ybArm->getArmJoint(joint_id).getData(sensed);
	lua_pushnumber(L, sensed.rpm );
	return 1;
}
static int lua_get_arm_current(lua_State *L) {
	static JointSensedCurrent sensed;
	if(!ybArm){return luaL_error(L,"Arm is not initialized!");}
	int joint_id = luaL_checkint(L, 1);
	ybArm->getArmJoint(joint_id).getData(sensed);
	lua_pushnumber(L, sensed.current.value() );
	return 1;
}

// Sets the spacing between the gripper fingers
static int lua_set_gripper_spacing(lua_State *L) {
	static GripperBarSpacingSetPoint barSpacing;
	double spacing = (double)lua_tonumber(L, 1);
	barSpacing.barSpacing = spacing * meter;
	ybArm->getArmGripper().setData(barSpacing);
	return 0;
}

// Gets the spacing between the gripper fingers
static int lua_get_gripper_spacing(lua_State *L) {
	static GripperSensedBarSpacing sensed;
	ybArm->getArmGripper().getData(sensed);
	lua_pushnumber(L, sensed.barSpacing.value() );
	return 1;
}

/* Sets the microstepping resolution
 * full step: 0, half step: 1, 256 microsteps: 8
 * For now, we keep the bars symmetric (bar1=bar2 for all params)
 */
static int lua_set_gripper_microsteps(lua_State *L) {
	static MicrostepResolution usteps;
	unsigned int res = (unsigned int)lua_tointeger(L, 1);
	usteps.setParameter(res);
	ybArm->getArmGripper().getGripperBar1().setConfigurationParameter(usteps);
	ybArm->getArmGripper().getGripperBar2().setConfigurationParameter(usteps);
	return 0;
}

// Sets the spacing between the gripper fingers
static int lua_get_gripper_microsteps(lua_State *L) {
	static MicrostepResolution usteps;
	static unsigned int res;
	//
	ybArm->getArmGripper().getGripperBar1().getConfigurationParameter(usteps);
	usteps.getParameter(res);
	lua_pushnumber(L, res );
	//
	ybArm->getArmGripper().getGripperBar2().getConfigurationParameter(usteps);
	usteps.getParameter(res);
	lua_pushnumber(L, res );
	return 2;
}

/* Limits */
static int lua_set_arm_max_positioning_velocity(lua_State *L) {
	static MaximumPositioningVelocity maxPositioningVelocity;
	int joint_id = luaL_checkint(L, 1);
	double max_velocity = (double)lua_tonumber(L, 2);
	maxPositioningVelocity
		.setParameter(max_velocity * radian_per_second);
	ybArm->getArmJoint(joint_id)
		.setConfigurationParameter(maxPositioningVelocity);
	return 0;
}

static int lua_get_arm_max_positioning_velocity(lua_State *L) {
	static MaximumPositioningVelocity maxPositioningVelocity;
	static quantity<angular_velocity> velocity;
	int joint_id = luaL_checkint(L, 1);
	ybArm->getArmJoint(joint_id)
		.getConfigurationParameter(maxPositioningVelocity);
	maxPositioningVelocity.getParameter(velocity);
	lua_pushnumber(L, velocity.value() );
	return 1;
}

static int lua_get_arm_joint_limit(lua_State *L) {
	static JointLimitsRadian jointLimitsRadian;
	static quantity<plane_angle> lowerLimit;
	static quantity<plane_angle> upperLimit;
	static bool areLimitsActive;
	int joint_id = luaL_checkint(L, 1);
	ybArm->getArmJoint(joint_id)
		.getConfigurationParameter(jointLimitsRadian);
	jointLimitsRadian
		.getParameter(lowerLimit,upperLimit,areLimitsActive);
	//
	lua_pushnumber(L, lowerLimit.value() );
	lua_pushnumber(L, upperLimit.value() );
	lua_pushboolean(L, areLimitsActive );
	return 3;
}

static int lua_set_arm_joint_limit(lua_State *L) {
	static JointLimitsRadian jointLimitsRadian;
	static quantity<plane_angle> lowerLimit;
	static quantity<plane_angle> upperLimit;
	int joint_id = luaL_checkint(L, 1);
	lowerLimit = luaL_checknumber(L, 2) * radian;
	upperLimit = luaL_checknumber(L, 3) * radian;
	bool areLimitsActive = (bool)lua_toboolean(L, 4);
	jointLimitsRadian.setParameter(lowerLimit,upperLimit,areLimitsActive);
	/*
		 ybArm->getArmJoint(joint_id)
		 .setConfigurationParameter(jointLimitsRadian);
		 */	
	return 0;
}

/* PID from JointConfigurator*/
static int lua_get_arm_pid1(lua_State *L) {
	static PParameterFirstParametersPositionControl p_param;
	static IParameterFirstParametersPositionControl i_param;
	static DParameterFirstParametersPositionControl d_param;
	static int p, i, d;
	//
	int joint_id = luaL_checkint(L, 1);
	ybArm->getArmJoint(joint_id)
		.getConfigurationParameter(p_param);
	p_param.getParameter(p);
	ybArm->getArmJoint(joint_id)
		.getConfigurationParameter(i_param);
	i_param.getParameter(i);
	ybArm->getArmJoint(joint_id)
		.getConfigurationParameter(d_param);
	d_param.getParameter(d);
	//
	lua_pushnumber(L, p );
	lua_pushnumber(L, i );
	lua_pushnumber(L, d );
	return 3;
}
static int lua_set_arm_pid1(lua_State *L) {
	static PParameterFirstParametersPositionControl p_param;
	static IParameterFirstParametersPositionControl i_param;
	static DParameterFirstParametersPositionControl d_param;
	//
	int joint_id = luaL_checkint(L, 1);
	int p = luaL_checkint(L, 2);
	int i = luaL_checkint(L, 3);
	int d = luaL_checkint(L, 4);
	p_param.setParameter(p);
	i_param.setParameter(i);
	d_param.setParameter(d);
	ybArm->getArmJoint(joint_id)
		.setConfigurationParameter(p_param);
	ybArm->getArmJoint(joint_id)
		.setConfigurationParameter(i_param);
	ybArm->getArmJoint(joint_id)
		.setConfigurationParameter(d_param);
	return 0;
}

/* PID from JointConfigurator*/
static int lua_get_arm_pid2(lua_State *L) {
	static PParameterSecondParametersPositionControl p_param;
	static IParameterSecondParametersPositionControl i_param;
	static DParameterSecondParametersPositionControl d_param;
	static int p, i, d;
	//
	int joint_id = luaL_checkint(L, 1);
	ybArm->getArmJoint(joint_id)
		.getConfigurationParameter(p_param);
	p_param.getParameter(p);
	ybArm->getArmJoint(joint_id)
		.getConfigurationParameter(i_param);
	i_param.getParameter(i);
	ybArm->getArmJoint(joint_id)
		.getConfigurationParameter(d_param);
	d_param.getParameter(d);
	//
	lua_pushnumber(L, p );
	lua_pushnumber(L, i );
	lua_pushnumber(L, d );
	return 3;
}
static int lua_set_arm_pid2(lua_State *L) {
	static PParameterSecondParametersPositionControl p_param;
	static IParameterSecondParametersPositionControl i_param;
	static DParameterSecondParametersPositionControl d_param;
	//
	int joint_id = luaL_checkint(L, 1);
	int p = luaL_checkint(L, 2);
	int i = luaL_checkint(L, 3);
	int d = luaL_checkint(L, 4);
	p_param.setParameter(p);
	i_param.setParameter(i);
	d_param.setParameter(d);
	ybArm->getArmJoint(joint_id)
		.setConfigurationParameter(p_param);
	ybArm->getArmJoint(joint_id)
		.setConfigurationParameter(i_param);
	ybArm->getArmJoint(joint_id)
		.setConfigurationParameter(d_param);
	return 0;
}

static const struct luaL_reg youbot_lib [] = {
	{"init_base", lua_init_base},
	{"init_arm", lua_init_arm},
	{"shutdown_base", lua_shutdown_base},
	{"shutdown_arm", lua_shutdown_arm},
	//
	{"calibrate_gripper", lua_calibrate_gripper},
	//
	{"set_base_velocity", lua_set_base_velocity},
	{"set_base_wheel", lua_set_base_wheel},
	{"get_base_position", lua_get_base_position},
	//
	{"set_arm_angle", lua_set_arm_angle},
	{"get_arm_position", lua_get_arm_position},
	{"get_arm_velocity", lua_get_arm_velocity},
	{"get_arm_torque", lua_get_arm_torque},
	{"get_arm_encoder", lua_get_arm_encoder},
	{"get_arm_rpm", lua_get_arm_rpm},
	{"get_arm_current", lua_get_arm_current},
	//
	{"get_arm_max_positioning_velocity", lua_get_arm_max_positioning_velocity},
	{"set_arm_max_positioning_velocity", lua_set_arm_max_positioning_velocity},
	{"get_arm_joint_limit", lua_get_arm_joint_limit},
	{"set_arm_joint_limit", lua_set_arm_joint_limit},
	//
	{"get_arm_pid1", lua_get_arm_pid1},
	{"set_arm_pid1", lua_set_arm_pid1},
	{"get_arm_pid2", lua_get_arm_pid2},
	{"set_arm_pid2", lua_set_arm_pid2},
	//
	{"set_gripper_spacing", lua_set_gripper_spacing},
	{"get_gripper_spacing", lua_get_gripper_spacing},
	{"get_gripper_microsteps", lua_get_gripper_microsteps},
	{"set_gripper_microsteps", lua_set_gripper_microsteps},
	//
	{NULL, NULL}
};

#ifdef __cplusplus
extern "C"
#endif
int luaopen_youbot (lua_State *L) {
#if LUA_VERSION_NUM == 502
	luaL_newlib(L, youbot_lib);
#else
	luaL_register(L, "youbot", youbot_lib);
#endif
	return 1;
}
