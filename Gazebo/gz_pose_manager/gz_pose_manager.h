#ifndef _GZ_POSE_MANAGER_H_
#define _GZ_POSE_MANAGER_H_

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <string>
#include <vector>
#include <msgpack.hpp>
#include "Comms/config.h"

extern "C" {
#include <zmq.h>
}

// gz_pose_manager.so : gazebo plugin for ground truth pose estimation
////////////////////////////////////////////////////////////////////////////////

namespace gazebo
{
class gz_pose_manager : public ModelPlugin
{
public:
  gz_pose_manager();
public:
  virtual ~gz_pose_manager();
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
private:
  void update();

private:
  physics::ModelPtr model;
private:
  physics::WorldPtr world;
private:
  event::ConnectionPtr update_connection;

private:
  void* context;
private:
  void* pose_socket;
private:
  std::string pose_channel_endpoint;
private:
  double pose_channel_rate;
private:
  double last_update_time;
};
}

#endif
