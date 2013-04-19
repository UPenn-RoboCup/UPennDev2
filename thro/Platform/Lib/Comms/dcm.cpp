#include "dcm.h"

// dcm : global interface for actuator and sensor data
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

Dcm::Dcm()
{
  // create shared memory segment for actuator and sensor data
  shared_memory_object::remove("dcm");
  dcm_segment = managed_shared_memory(create_only, "dcm", 65536);

  // get array lengths from config file
  Config config;

  std::vector<std::string> joint_id = config.get_string_vector("joint.id");
  n_joint = joint_id.size();

  std::vector<std::string> motor_id = config.get_string_vector("motor.id");
  n_motor = motor_id.size();

  std::vector<std::string> force_torque_id = config.get_string_vector("force_torque.id");
  n_force_torque = force_torque_id.size();

  std::vector<std::string> ahrs_id = config.get_string_vector("ahrs.id");
  n_ahrs = ahrs_id.size();

  std::vector<std::string> battery_id = config.get_string_vector("battery.id");
  n_battery = battery_id.size();

  // create shared data objects
  joint_enable = 
    dcm_segment.construct<double>("joint_enable")[n_joint](0);
  joint_p_gain = 
    dcm_segment.construct<double>("joint_p_gain")[n_joint](0);
  joint_i_gain = 
    dcm_segment.construct<double>("joint_i_gain")[n_joint](0);
  joint_d_gain = 
    dcm_segment.construct<double>("joint_d_gain")[n_joint](0);
  joint_force = 
    dcm_segment.construct<double>("joint_force")[n_joint](0);
  joint_position = 
    dcm_segment.construct<double>("joint_position")[n_joint](0);
  joint_velocity = 
    dcm_segment.construct<double>("joint_velocity")[n_joint](0);
  joint_force_sensor = 
    dcm_segment.construct<double>("joint_force_sensor")[n_joint](0);
  joint_position_sensor = 
    dcm_segment.construct<double>("joint_position_sensor")[n_joint](0);
  joint_velocity_sensor = 
    dcm_segment.construct<double>("joint_velocity_sensor")[n_joint](0);
  motor_force_sensor = 
    dcm_segment.construct<double>("motor_force_sensor")[n_motor](0);
  motor_position_sensor = 
    dcm_segment.construct<double>("motor_position_sensor")[n_motor](0);
  motor_velocity_sensor = 
    dcm_segment.construct<double>("motor_velocity_sensor")[n_motor](0);
  motor_current_sensor = 
    dcm_segment.construct<double>("motor_current_sensor")[n_motor](0);
  motor_temperature_sensor = 
    dcm_segment.construct<double>("motor_temperature_sensor")[n_motor](0);
  force_torque = 
    dcm_segment.construct<double>("force_torque")[n_force_torque](0);
  ahrs = 
    dcm_segment.construct<double>("ahrs")[n_ahrs](0);
  battery = 
    dcm_segment.construct<double>("battery")[n_battery](0);

  // create shared update flags
  joint_enable_updated = 
    dcm_segment.construct<double>("joint_enable_updated")[n_joint](0);
  joint_p_gain_updated = 
    dcm_segment.construct<double>("joint_p_gain_updated")[n_joint](0);
  joint_i_gain_updated = 
    dcm_segment.construct<double>("joint_i_gain_updated")[n_joint](0);
  joint_d_gain_updated = 
    dcm_segment.construct<double>("joint_d_gain_updated")[n_joint](0);
  joint_force_updated = 
    dcm_segment.construct<double>("joint_force_updated")[n_joint](0);
  joint_position_updated = 
    dcm_segment.construct<double>("joint_position_updated")[n_joint](0);
  joint_velocity_updated = 
    dcm_segment.construct<double>("joint_velocity_updated")[n_joint](0);
  joint_force_sensor_updated = 
    dcm_segment.construct<double>("joint_force_sensor_updated")[n_joint](0);
  joint_position_sensor_updated = 
    dcm_segment.construct<double>("joint_position_sensor_updated")[n_joint](0);
  joint_velocity_sensor_updated = 
    dcm_segment.construct<double>("joint_velocity_sensor_updated")[n_joint](0);
  motor_force_sensor_updated = 
    dcm_segment.construct<double>("motor_force_sensor_updated")[n_motor](0);
  motor_position_sensor_updated = 
    dcm_segment.construct<double>("motor_position_sensor_updated")[n_motor](0);
  motor_velocity_sensor_updated = 
    dcm_segment.construct<double>("motor_velocity_sensor_updated")[n_motor](0);
  motor_current_sensor_updated = 
    dcm_segment.construct<double>("motor_current_sensor_updated")[n_motor](0);
  motor_temperature_sensor_updated = 
    dcm_segment.construct<double>("motor_temperature_sensor_updated")[n_motor](0);
  force_torque_updated = 
    dcm_segment.construct<double>("force_torque_updated")[n_force_torque](0);
  ahrs_updated = 
    dcm_segment.construct<double>("ahrs_updated")[n_ahrs](0);
  battery_updated = 
    dcm_segment.construct<double>("battery_updated")[n_battery](0);
}

Dcm::~Dcm()
{
  shared_memory_object::remove("dcm");
}
