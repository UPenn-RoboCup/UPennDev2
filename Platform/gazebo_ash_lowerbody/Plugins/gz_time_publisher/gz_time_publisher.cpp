#include "gz_time_publisher.h"

// gz_time_publisher.so : gazebo plugin for joint control and proprioception
////////////////////////////////////////////////////////////////////////////////

// Mike Hopkins 3/20/13

namespace gazebo
{
  gz_time_publisher::gz_time_publisher()
  {
    this->endpoint = "tcp://127.0.0.1:12000";
    this->desired_update_rate = 500.0;
  }

  gz_time_publisher::~gz_time_publisher()
  {
    event::Events::DisconnectWorldUpdateBegin(this->update_connection);
    zmq_close(this->socket);
    zmq_ctx_destroy(this->context);
  }

  void gz_time_publisher::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  {
    // initialize pointers
    this->world = _world; 

    // initialize sim time
    this->last_update_time = this->world->GetSimTime().Double();

    // initialize sim time socket
    this->context = zmq_ctx_new();
    this->socket = zmq_socket(this->context, ZMQ_PUB);
    zmq_bind(this->socket, this->endpoint.c_str());

    // initialize callback functions
    this->update_connection = event::Events::Events::ConnectWorldUpdateBegin(
      boost::bind(&gz_time_publisher::update, this));
  }

  void gz_time_publisher::update()
  {
    // update sim time
    double current_time = this->world->GetSimTime().Double();
    double dt = current_time - this->last_update_time;

    // publish sim time message
    if (dt >= 1/this->desired_update_rate)
    {
      // create time data vector
      std::vector<double> time_data;
      time_data.push_back(current_time);
      time_data.push_back(dt);
 
      // serialize it
      msgpack::sbuffer time_buffer;
      msgpack::pack(time_buffer, time_data);
  
      // publish current sim time
      zmq_msg_t time_msg;
      zmq_msg_init_size(&time_msg, time_buffer.size() + 4);

      memcpy(zmq_msg_data(&time_msg), "time", 4);
      memcpy((char *)zmq_msg_data(&time_msg) + 4,
        time_buffer.data(), time_buffer.size());

      zmq_msg_send(&time_msg, socket, 0);
      zmq_msg_close(&time_msg);

      this->last_update_time = current_time;
    }
  }

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(gz_time_publisher)
}
