#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <vector>
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
#include "shared_data.h"

// shared_data : utilities for shared actuator and sensor data 
// author: Mike Hopkins
///////////////////////////////////////////////////////////////////////////

static std::vector<double> lua_getvector(lua_State *L, int index);
static void load_bias_settings();

namespace shared_data {
  struct actuator_data actuator;
  struct sensor_data sensor;
  struct bias_data bias;
};

void shared_data::entry()
{
  // create shared memory segments for actuator and sensor data
  using namespace boost::interprocess;

  shared_memory_object::remove("actuator");
  static managed_shared_memory actuator_segment(create_only, "actuator", 65536);

  shared_memory_object::remove("sensor");
  static managed_shared_memory sensor_segment(create_only, "sensor", 65536);

  // create actuator objects
  actuator.joint_write_access = 
    actuator_segment.construct<double>("joint_write_access")[N_JOINT](0);
  actuator.joint_enable =
    actuator_segment.construct<double>("joint_enable")[N_JOINT](0);
  actuator.joint_stiffness = 
    actuator_segment.construct<double>("joint_stiffness")[N_JOINT](0);
  actuator.joint_damping = 
    actuator_segment.construct<double>("joint_damping")[N_JOINT](0);
  actuator.joint_force = 
    actuator_segment.construct<double>("joint_force")[N_JOINT](0);
  actuator.joint_position =
    actuator_segment.construct<double>("joint_position")[N_JOINT](0);
  actuator.joint_velocity = 
    actuator_segment.construct<double>("joint_velocity")[N_JOINT](0);

  // create actuator update flags
  actuator.joint_write_access_updated = 
    actuator_segment.construct<double>("joint_write_access_updated")[N_JOINT](0);
  actuator.joint_enable_updated =
    actuator_segment.construct<double>("joint_enable_updated")[N_JOINT](0);
  actuator.joint_stiffness_updated = 
    actuator_segment.construct<double>("joint_stiffness_updated")[N_JOINT](0);
  actuator.joint_damping_updated = 
    actuator_segment.construct<double>("joint_damping_updated")[N_JOINT](0);
  actuator.joint_force_updated = 
    actuator_segment.construct<double>("joint_force_updated")[N_JOINT](0);
  actuator.joint_position_updated =
    actuator_segment.construct<double>("joint_position_updated")[N_JOINT](0);
  actuator.joint_velocity_updated = 
    actuator_segment.construct<double>("joint_velocity_updated")[N_JOINT](0);

  // create sensor objects
  sensor.joint_force = 
    sensor_segment.construct<double>("joint_force")[N_JOINT](0);
  sensor.joint_position = 
    sensor_segment.construct<double>("joint_position")[N_JOINT](0);
  sensor.joint_velocity = 
    sensor_segment.construct<double>("joint_velocity")[N_JOINT](0);
  sensor.motor_force = 
    sensor_segment.construct<double>("motor_force")[N_MOTOR](0);
  sensor.motor_position = 
    sensor_segment.construct<double>("motor_position")[N_MOTOR](0);
  sensor.motor_velocity = 
    sensor_segment.construct<double>("motor_velocity")[N_MOTOR](0);
  sensor.motor_current = 
    sensor_segment.construct<double>("motor_current")[N_MOTOR](0);
  sensor.motor_temperature = 
    sensor_segment.construct<double>("motor_temperature")[N_MOTOR](0);
  sensor.force_torque = 
    sensor_segment.construct<double>("force_torque")[N_FORCE_TORQUE](0);
  sensor.tactile_array = 
    sensor_segment.construct<double>("tactile_array")[N_TACTILE_ARRAY](0);
  sensor.ahrs = 
    sensor_segment.construct<double>("ahrs")[N_AHRS](0);
  sensor.battery = 
    sensor_segment.construct<double>("battery")[N_BATTERY](0);

  // create sensor update flag
  sensor.joint_force_updated = 
    sensor_segment.construct<double>("joint_force_updated")[N_JOINT](0);
  sensor.joint_position_updated = 
    sensor_segment.construct<double>("joint_position_updated")[N_JOINT](0);
  sensor.joint_velocity_updated = 
    sensor_segment.construct<double>("joint_velocity_updated")[N_JOINT](0);
  sensor.motor_force_updated = 
    sensor_segment.construct<double>("motor_force_updated")[N_MOTOR](0);
  sensor.motor_position_updated = 
    sensor_segment.construct<double>("motor_position_updated")[N_MOTOR](0);
  sensor.motor_velocity_updated = 
    sensor_segment.construct<double>("motor_velocity_updated")[N_MOTOR](0);
  sensor.motor_current_updated = 
    sensor_segment.construct<double>("motor_current_updated")[N_MOTOR](0);
  sensor.motor_temperature_updated = 
    sensor_segment.construct<double>("motor_temperature_updated")[N_MOTOR](0);
  sensor.force_torque_updated = 
    sensor_segment.construct<double>("force_torque_updated")[N_FORCE_TORQUE](0);
  sensor.tactile_array_updated = 
    sensor_segment.construct<double>("tactile_array_updated")[N_TACTILE_ARRAY](0);
  sensor.ahrs_updated = 
    sensor_segment.construct<double>("ahrs_updated")[N_AHRS](0);
  sensor.battery_updated = 
    sensor_segment.construct<double>("battery_updated")[N_BATTERY](0);

  // load bias settings from lua config file
  load_bias_settings();
}

void shared_data::exit()
{
  // destroy shared memory segments
  using namespace boost::interprocess;

  shared_memory_object::remove("actuator");
  shared_memory_object::remove("sensor");
}

static std::vector<double> lua_getvector(lua_State *L, int index)
{
  // get double vector from lua stack
  std::vector<double> vec(0);
  if (!lua_istable(L, index))
    return vec;
  size_t len = lua_objlen(L, index);
  vec.resize(len);
  if (index < 0)
    --index;

  for (int i = 0; i < len; i++)
  {
    lua_pushinteger(L, i + 1);
    lua_gettable(L, index);
    vec[i] = luaL_checknumber(L, -1);
    lua_pop(L, 1);
  }
  return vec;
}

static void load_bias_settings()
{
  // load bias settings from Config.lua module
  using namespace shared_data;

  lua_State *L = luaL_newstate();
  luaL_openlibs(L);
  if (luaL_loadfile(L, "include.lua") || lua_pcall(L, 0, 0, 0))
  {
    fprintf(stderr, "%s", lua_tostring(L, -1));
    exit(EXIT_FAILURE);
  }
  if (luaL_loadstring(L, "require('Config')") || lua_pcall(L, 0, 0, 0))
  {
    fprintf(stderr, "%s", lua_tostring(L, -1));
    exit(EXIT_FAILURE);
  }
  lua_getglobal(L, "Config");
  if (!lua_istable(L, -1))
  {
    fprintf(stderr, "unable to load Config table from Config.lua");
    exit(EXIT_FAILURE);
  }
  lua_getfield(L, -1, "bias");
  if (!lua_istable(L, -1))
  {
    fprintf(stderr, "unable to load Config.bias table from Config.lua");
    exit(EXIT_FAILURE);
  }

  lua_getfield(L, -1, "joint_position_bias");
  static std::vector<double> joint_position_bias = lua_getvector(L, -1);
  joint_position_bias.resize(N_JOINT, 0);
  lua_pop(L, 1);

  lua_getfield(L, -1, "joint_force_bias");
  static std::vector<double> joint_force_bias = lua_getvector(L, -1);
  joint_force_bias.resize(N_JOINT, 0);
  lua_pop(L, 1);

  lua_getfield(L, -1, "motor_position_bias");
  static std::vector<double> motor_position_bias = lua_getvector(L, -1);
  motor_position_bias.resize(N_MOTOR, 0);
  lua_pop(L, 1);

  lua_getfield(L, -1, "motor_force_bias");
  static std::vector<double> motor_force_bias = lua_getvector(L, -1);
  motor_force_bias.resize(N_MOTOR, 0);
  lua_pop(L, 1);

  lua_getfield(L, -1, "force_torque_bias");
  static std::vector<double> force_torque_bias = lua_getvector(L, -1);
  force_torque_bias.resize(N_FORCE_TORQUE, 0);
  lua_pop(L, 1);

  lua_getfield(L, -1, "tactile_array_bias");
  static std::vector<double> tactile_array_bias = lua_getvector(L, -1);
  tactile_array_bias.resize(N_TACTILE_ARRAY, 0);
  lua_pop(L, 1);

  // map bias values to global struct
  bias.joint_position = &joint_position_bias[0];
  bias.joint_force = &joint_force_bias[0];
  bias.motor_position = &motor_position_bias[0];
  bias.motor_force = &motor_force_bias[0];
  bias.force_torque = &force_torque_bias[0];
  bias.tactile_array = &tactile_array_bias[0];

  lua_close(L);
}
