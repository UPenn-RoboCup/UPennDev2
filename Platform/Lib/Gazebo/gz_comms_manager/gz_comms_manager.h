#ifndef _GZ_COMMS_MANAGER_H_
#define _GZ_COMMS_MANAGER_H_

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/Sensor.hh>

#include <string>
#include <vector>
#include "config.h"
#include "dcm.h"

extern "C" {
#include "pid.h"
#include "filter.h"
}

// gz_comms_manager.so : gazebo plugin for joint control and proprioception
////////////////////////////////////////////////////////////////////////////////

namespace gazebo
{
  class gz_comms_manager : public ModelPlugin
  {
    public: gz_comms_manager();
    public: virtual ~gz_comms_manager();
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    private: void initialize_controllers();
    private: void reset();
    private: void update();

    private: physics::ModelPtr model;
    private: physics::WorldPtr world;
    private: event::ConnectionPtr update_connection;
    private: event::ConnectionPtr reset_connection;
    private: common::Time last_update_time;
    private: common::Time dynamics_time_step;

    // Joints
    private: physics::Joint_V joints;
    private: std::vector<std::string> joint_id;

    // Controller structs
    private: std::vector<struct pid> joint_position_pids;
    private: std::vector<struct filter> joint_velocity_filters;

    // Force torque joints
    private: unsigned int l_ankle_index;
    private: unsigned int r_ankle_index;
    private: unsigned int l_wrist_index;
    private: unsigned int r_wrist_index;

    // Imu
    private: boost::shared_ptr<sensors::ImuSensor> imu_sensor;
    private: std::string imu_link_name;
    private: std::string imu_sensor_name;

    // Controller settings
    private: int joint_max;
    private: double p_gain_constant;
    private: double i_gain_constant;
    private: double d_gain_constant;
    private: double d_break_freq;
    private: double p_gain_default;
    private: double i_gain_default;
    private: double d_gain_default;
  };
}

#endif
