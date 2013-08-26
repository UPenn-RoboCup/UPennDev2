#include "gz_pose_manager.h"

// gz_pose_manager.so : gazebo plugin for ground truth pose estimation
////////////////////////////////////////////////////////////////////////////////

namespace gazebo
{
  gz_pose_manager::gz_pose_manager()
  {
    this->pose_channel_endpoint = "tcp://127.0.0.1:12012";
    this->pose_channel_rate = 1000;
  }

  gz_pose_manager::~gz_pose_manager()
  {
    event::Events::DisconnectWorldUpdateBegin(this->update_connection);
    zmq_close(this->pose_socket);
    zmq_ctx_destroy(this->context);
  }

  void gz_pose_manager::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // initialize pointers
    this->model = _parent;
    this->world = this->model->GetWorld(); 

    // initialize sim time
    this->last_update_time = this->world->GetSimTime().Double();

    // initialize sim time socket
    this->context = zmq_ctx_new();
    this->pose_socket = zmq_socket(this->context, ZMQ_PUB);
    zmq_bind(this->pose_socket, this->pose_channel_endpoint.c_str());

    this->update_connection = event::Events::Events::ConnectWorldUpdateBegin(
      boost::bind(&gz_pose_manager::update, this));
  }

  void gz_pose_manager::update()
  {
    // update sim time
    double current_time = this->world->GetSimTime().Double();
    double dt = current_time - this->last_update_time;
 
    // update ground truth pose
    gazebo::physics::LinkPtr torso = this->model->GetLink("chest");
    math::Pose torso_pose = torso->GetWorldPose();
    math::Vector3 torso_position = torso_pose.pos;
    math::Vector3 torso_euler = torso_pose.rot.GetAsEuler();

    // create pose data type
    std::vector<double> pose_t(7);
    pose_t[0] = torso_position[0];
    pose_t[1] = torso_position[1];
    pose_t[2] = torso_position[2];
    pose_t[3] = torso_euler[0];
    pose_t[4] = torso_euler[1];
    pose_t[5] = torso_euler[2];
    pose_t[6] = current_time;

    // publish pose message
    if (dt >= (1/this->pose_channel_rate - 1e-6))
    {
      // serialize current sim time
      msgpack::sbuffer pose_buffer;
      msgpack::pack(pose_buffer, pose_t);
  
      // broadcast message on "pose" channel
      zmq_msg_t pose_msg;
      zmq_msg_init_size(&pose_msg, pose_buffer.size() + 4);

      memcpy(zmq_msg_data(&pose_msg), "pose", 4);
      memcpy((char *)zmq_msg_data(&pose_msg) + 4,
        pose_buffer.data(), pose_buffer.size());

      zmq_msg_send(&pose_msg, pose_socket, 0);
      zmq_msg_close(&pose_msg);

      this->last_update_time = current_time;
    }
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(gz_pose_manager)
}
