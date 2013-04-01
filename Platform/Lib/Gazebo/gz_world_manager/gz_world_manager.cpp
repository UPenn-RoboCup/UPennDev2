#include "gz_world_manager.h"

// gz_world_manager.so : gazebo plugin for joint control and proprioception
////////////////////////////////////////////////////////////////////////////////

// Mike Hopkins 3/20/13

namespace gazebo
{
  gz_world_manager::gz_world_manager()
  {
    this->physics_time_step = 0.000334;
    this->time_channel_endpoint = "tcp://127.0.0.1:12000";
    this->time_channel_rate = 1000;
  }

  gz_world_manager::~gz_world_manager()
  {
    event::Events::DisconnectWorldUpdateBegin(this->update_connection);
    zmq_close(this->time_socket);
    zmq_ctx_destroy(this->context);
  }

  void gz_world_manager::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // initialize pointers
    this->world = _world; 

    // load config parameters
    Config config;
    this->physics_time_step =
      config.get_double("world_manager.physics_time_step");
    this->time_channel_endpoint =
      config.get_string("world_manager.time_channel_endpoint");
    this->time_channel_rate =
      config.get_double("world_manager.time_channel_rate");

    // initialize sim physics time step
    this->world->GetPhysicsEngine()->SetStepTime(this->physics_time_step);

    // initialize sim time
    this->last_update_time = this->world->GetSimTime().Double();

    // initialize sim time socket
    this->context = zmq_ctx_new();
    this->time_socket = zmq_socket(this->context, ZMQ_PUB);
    zmq_bind(this->time_socket, this->time_channel_endpoint.c_str());

    // initialize callback functions
    this->update_connection = event::Events::Events::ConnectWorldUpdateBegin(
      boost::bind(&gz_world_manager::update, this));
  }

  void gz_world_manager::update()
  {
    // update sim time
    double current_time = this->world->GetSimTime().Double();
    double dt = current_time - this->last_update_time;

    // publish sim time message
    if (dt >= (1/this->time_channel_rate - 1e-6))
    {
      // serialize current sim time
      msgpack::sbuffer time_buffer;
      msgpack::pack(time_buffer, current_time);
  
      // broadcast message on "time" channel
      zmq_msg_t time_msg;
      zmq_msg_init_size(&time_msg, time_buffer.size() + 4);

      memcpy(zmq_msg_data(&time_msg), "time", 4);
      memcpy((char *)zmq_msg_data(&time_msg) + 4,
        time_buffer.data(), time_buffer.size());

      zmq_msg_send(&time_msg, time_socket, 0);
      zmq_msg_close(&time_msg);

      this->last_update_time = current_time;
    }
  }

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(gz_world_manager)
}
