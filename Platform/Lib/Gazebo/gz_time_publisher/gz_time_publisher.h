#ifndef _GZ_TIME_PUBLISHER_H_
#define _GZ_TIME_PUBLISHER_H_

#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <string>
#include <vector>
#include <msgpack.hpp>

extern "C" {
#include <zmq.h>
}

// gz_time_publisher.so : gazebo plugin for publishing simulation time
////////////////////////////////////////////////////////////////////////////////

namespace gazebo
{
  class gz_time_publisher : public WorldPlugin
  {
    public: gz_time_publisher();
    public: virtual ~gz_time_publisher();
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
    private: void update();

    private: physics::WorldPtr world;
    private: event::ConnectionPtr update_connection;

    private: void *context;
    private: void *socket;
    private: std::string endpoint;

    private: double last_update_time;
    private: double desired_update_rate;
  };
}

#endif
