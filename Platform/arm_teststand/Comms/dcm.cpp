#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include "dcm.h"

// dcm : global interface for actuator and sensor data
// author : Mike Hopkins
///////////////////////////////////////////////////////////////////////////

// create global dcm instance
Dcm dcm;

Dcm::Dcm()
{
  // create shared memory segment for actuator and sensor data
  using namespace boost::interprocess;

  shared_memory_object::remove("dcm");
  static managed_shared_memory dcm_segment(create_only, "dcm", 65536);

  // create shared data objects
  joint_enable = 
    dcm_segment.construct<double>("joint_enable")[N_JOINT](0);
  joint_p_gain = 
    dcm_segment.construct<double>("joint_p_gain")[N_JOINT](0);
  joint_i_gain = 
    dcm_segment.construct<double>("joint_i_gain")[N_JOINT](0);
  joint_d_gain = 
    dcm_segment.construct<double>("joint_d_gain")[N_JOINT](0);
  joint_force = 
    dcm_segment.construct<double>("joint_force")[N_JOINT](0);
  joint_position = 
    dcm_segment.construct<double>("joint_position")[N_JOINT](0);
  joint_velocity = 
    dcm_segment.construct<double>("joint_velocity")[N_JOINT](0);
  joint_force_sensor = 
    dcm_segment.construct<double>("joint_force_sensor")[N_JOINT](0);
  joint_position_sensor = 
    dcm_segment.construct<double>("joint_position_sensor")[N_JOINT](0);
  joint_velocity_sensor = 
    dcm_segment.construct<double>("joint_velocity_sensor")[N_JOINT](0);
  motor_force_sensor = 
    dcm_segment.construct<double>("motor_force_sensor")[N_MOTOR](0);
  motor_position_sensor = 
    dcm_segment.construct<double>("motor_position_sensor")[N_MOTOR](0);
  motor_velocity_sensor = 
    dcm_segment.construct<double>("motor_velocity_sensor")[N_MOTOR](0);
  motor_current_sensor = 
    dcm_segment.construct<double>("motor_current_sensor")[N_MOTOR](0);
  motor_temperature_sensor = 
    dcm_segment.construct<double>("motor_temperature_sensor")[N_MOTOR](0);
  force_torque = 
    dcm_segment.construct<double>("force_torque")[N_FORCE_TORQUE](0);
  ahrs = 
    dcm_segment.construct<double>("ahrs")[N_AHRS](0);
  battery = 
    dcm_segment.construct<double>("battery")[N_BATTERY](0);

  // create shared update flags
  joint_enable_updated = 
    dcm_segment.construct<double>("joint_enable_updated")[N_JOINT](0);
  joint_p_gain_updated = 
    dcm_segment.construct<double>("joint_p_gain_updated")[N_JOINT](0);
  joint_i_gain_updated = 
    dcm_segment.construct<double>("joint_i_gain_updated")[N_JOINT](0);
  joint_d_gain_updated = 
    dcm_segment.construct<double>("joint_d_gain_updated")[N_JOINT](0);
  joint_force_updated = 
    dcm_segment.construct<double>("joint_force_updated")[N_JOINT](0);
  joint_position_updated = 
    dcm_segment.construct<double>("joint_position_updated")[N_JOINT](0);
  joint_velocity_updated = 
    dcm_segment.construct<double>("joint_velocity_updated")[N_JOINT](0);
  joint_force_sensor_updated = 
    dcm_segment.construct<double>("joint_force_sensor_updated")[N_JOINT](0);
  joint_position_sensor_updated = 
    dcm_segment.construct<double>("joint_position_sensor_updated")[N_JOINT](0);
  joint_velocity_sensor_updated = 
    dcm_segment.construct<double>("joint_velocity_sensor_updated")[N_JOINT](0);
  motor_force_sensor_updated = 
    dcm_segment.construct<double>("motor_force_sensor_updated")[N_MOTOR](0);
  motor_position_sensor_updated = 
    dcm_segment.construct<double>("motor_position_sensor_updated")[N_MOTOR](0);
  motor_velocity_sensor_updated = 
    dcm_segment.construct<double>("motor_velocity_sensor_updated")[N_MOTOR](0);
  motor_current_sensor_updated = 
    dcm_segment.construct<double>("motor_current_sensor_updated")[N_MOTOR](0);
  motor_temperature_sensor_updated = 
    dcm_segment.construct<double>("motor_temperature_sensor_updated")[N_MOTOR](0);
  force_torque_updated = 
    dcm_segment.construct<double>("force_torque_updated")[N_FORCE_TORQUE](0);
  ahrs_updated = 
    dcm_segment.construct<double>("ahrs_updated")[N_AHRS](0);
  battery_updated = 
    dcm_segment.construct<double>("battery_updated")[N_BATTERY](0);
}

Dcm::~Dcm()
{
  // destroy shared memory segment
  using namespace boost::interprocess;
  shared_memory_object::remove("dcm");
}
