#ifndef _GZ_WORLD_MANAGER_H_
#define _GZ_WORLD_MANAGER_H_

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <string>
#include <vector>
#include <msgpack.hpp>
#include "config.h"

extern "C" {
#include <zmq.h>
}

// gz_world_manager.so : gazebo plugin for publishing simulation time
////////////////////////////////////////////////////////////////////////////////

namespace gazebo
{
  class gz_world_manager : public WorldPlugin
  {
    public: gz_world_manager();
    public: virtual ~gz_world_manager();
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
    private: void update();

    private: physics::WorldPtr world;
    private: event::ConnectionPtr update_connection;
    private: double physics_time_step;

    private: void *context;
    private: void *time_socket;
    private: std::string time_channel_endpoint;
    private: double time_channel_rate;
    private: double last_update_time;
  };
}

#endif
