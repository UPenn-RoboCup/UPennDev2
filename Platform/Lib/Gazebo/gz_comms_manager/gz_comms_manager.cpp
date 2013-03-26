#include "gz_comms_manager.h"

// gz_comms_manager.so : gazebo plugin for joint control and proprioception
////////////////////////////////////////////////////////////////////////////////

// Mike Hopkins 3/20/13

extern Dcm dcm;

namespace gazebo
{
  gz_comms_manager::gz_comms_manager()
  {
    // initialize sensor id's
    this->imu_link_name = "torso";
    this->imu_sensor_name = "imu_sensor";

    // load config parameters
    Config config;
    this->p_gain_constant = config.get_double("comms_manager.p_gain_constant");
    this->i_gain_constant = config.get_double("comms_manager.i_gain_constant");
    this->d_gain_constant = config.get_double("comms_manager.d_gain_constant");
    this->d_break_freq = config.get_double("comms_manager.d_break_freq");
    this->p_gain_default = config.get_double("comms_manager.p_gain_default");
    this->i_gain_default = config.get_double("comms_manager.i_gain_default");
    this->d_gain_default = config.get_double("comms_manager.d_gain_default");
    this->joint_max = config.get_int("comms_manager.joint_max");

    this->joint_id = config.get_string_vector("joint.id");
    this->joint_id.resize(this->joint_max);

    // initialize force-torque joints
    this->l_ankle_index = -1;
    this->r_ankle_index = -1;
    this->l_wrist_index = -1;
    this->r_wrist_index = -1;

    for (int i = 0; i < this->joint_id.size(); i++)
    {
      if (joint_id[i] == "l_ankle_roll")
        this->l_ankle_index = i; 
      if (joint_id[i] == "r_ankle_roll")
        this->r_ankle_index = i; 
      if (joint_id[i] == "l_wrist_roll")
        this->l_wrist_index = i; 
      if (joint_id[i] == "r_wrist_roll")
        this->r_wrist_index = i; 
    }
  }

  gz_comms_manager::~gz_comms_manager()
  {
    event::Events::DisconnectWorldUpdateBegin(this->update_connection);
    event::Events::DisconnectStop(this->reset_connection);
  }

  void gz_comms_manager::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // initialize pointers
    this->model = _parent;
    this->world = this->model->GetWorld(); 

    // initialize time step
    this->dynamics_time_step = this->world->GetPhysicsEngine()->GetStepTime();

    // initialize joints
    this->joints.resize(this->joint_id.size());
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    { 
      this->joints[i] = this->model->GetJoint(this->joint_id[i]);
      if (!this->joints[i])
      { 
        gzerr << "Error: joint [" << this->joint_id[i] << "] not present\n";
        return;
      }

      dcm.joint_p_gain[i] = this->p_gain_default;
      dcm.joint_i_gain[i] = this->i_gain_default;
      dcm.joint_d_gain[i] = this->d_gain_default;
      dcm.joint_position[i] = 0;
      dcm.joint_velocity[i] = 0;
      dcm.joint_force[i] = 0;
    }

    // initialize joint controllers
    this->initialize_controllers();

    // initialize sensors
    this->imu_sensor = boost::shared_dynamic_cast<sensors::ImuSensor>(
      sensors::SensorManager::Instance()->GetSensor(
        this->world->GetName()
        + "::" + this->model->GetScopedName()
        + "::" + this->imu_link_name
        + "::" + this->imu_sensor_name
      )
    );
    if (!this->imu_sensor)
      gzerr << "imu_sensor not found\n";

    // initialize callback functions
    this->update_connection = event::Events::Events::ConnectWorldUpdateBegin(
      boost::bind(&gz_comms_manager::update, this));
    this->reset_connection = event::Events::Events::ConnectStop(
      boost::bind(&gz_comms_manager::reset, this));
  }

  void gz_comms_manager::initialize_controllers()
  {
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    { 
      // get joint limits
      double max_force = this->joints[i]->GetEffortLimit(0);
      double min_position = this->joints[i]->GetLowStop(0).Radian();
      double max_position = this->joints[i]->GetHighStop(0).Radian();

      // initialize position pid controller
      struct pid position_pid = new_pid(this->dynamics_time_step.Double());
      pid_set_setpoint_limits(&position_pid, min_position, max_position);
      pid_set_output_limits(&position_pid, -max_force, max_force);

      // initialize velocity filter
      struct filter velocity_filter = 
        new_low_pass(this->dynamics_time_step.Double(), this->d_break_freq);
      filter_set_output_limits(&velocity_filter, -max_force, max_force);

      // update joint structs
      if (i < joint_position_pids.size())
        this->joint_position_pids[i] = position_pid;
      else
        this->joint_position_pids.push_back(position_pid);

      if (i < joint_velocity_filters.size())
        this->joint_velocity_filters[i] = velocity_filter;
      else
        this->joint_velocity_filters.push_back(velocity_filter);
    }
  }

  void gz_comms_manager::reset()
  {
    for (unsigned int i = 0; i < this->joints.size(); ++i)
    { 
      dcm.joint_p_gain[i] = this->p_gain_default;
      dcm.joint_i_gain[i] = this->i_gain_default; 
      dcm.joint_d_gain[i] = this->d_gain_default; 
      dcm.joint_position[i] = 0;
      dcm.joint_velocity[i] = 0;
      dcm.joint_force[i] = 0;
    }
    this->initialize_controllers();
  }

  void gz_comms_manager::update()
  {
    // update force-torque sensors
    {
      physics::JointWrench l_ankle_wrench, r_ankle_wrench;
      physics::JointWrench l_wrist_wrench, r_wrist_wrench;

      unsigned int index0 = 0;
      if (l_ankle_index != -1)
        l_ankle_wrench = this->joints[l_ankle_index]->GetForceTorque(index0);
      if (r_ankle_index != -1)
        r_ankle_wrench = this->joints[r_ankle_index]->GetForceTorque(index0);
      if (l_wrist_index != -1)
        l_wrist_wrench = this->joints[l_wrist_index]->GetForceTorque(index0);
      if (r_wrist_index != -1)
        r_wrist_wrench = this->joints[r_wrist_index]->GetForceTorque(index0);

      math::Vector3 l_ankle_force = l_ankle_wrench.body1Force;
      math::Vector3 l_ankle_torque = l_ankle_wrench.body1Torque;
      math::Vector3 r_ankle_force = r_ankle_wrench.body1Force;
      math::Vector3 r_ankle_torque = r_ankle_wrench.body1Torque;
      math::Vector3 l_wrist_force = l_wrist_wrench.body1Force;
      math::Vector3 l_wrist_torque = l_wrist_wrench.body1Torque;
      math::Vector3 r_wrist_force = r_wrist_wrench.body1Force;
      math::Vector3 r_wrist_torque = r_wrist_wrench.body1Torque;

      for (int i = 0; i < 3; i++)
      {
        dcm.force_torque[i] = l_ankle_force[i];
        dcm.force_torque[i+3] = l_ankle_torque[i];
        dcm.force_torque[i+6] = r_ankle_force[i];
        dcm.force_torque[i+9] = r_ankle_torque[i];
        dcm.force_torque[i+12] = l_wrist_force[i];
        dcm.force_torque[i+15] = l_wrist_torque[i];
        dcm.force_torque[i+18] = r_wrist_force[i];
        dcm.force_torque[i+21] = r_wrist_torque[i];
      }
    }

    // update imu sensor
    {
      math::Vector3 gyro = this->imu_sensor->GetAngularVelocity();
      math::Vector3 accel = this->imu_sensor->GetLinearAcceleration();
      math::Vector3 euler = this->imu_sensor->GetOrientation().GetAsEuler();
      for (int i = 0; i < 3; i++)
      {
        dcm.ahrs[i] = gyro[i];
        dcm.ahrs[i+3] = accel[i];
        dcm.ahrs[i+6] = euler[i];
      }
    }

    // reset joint controllers if time step is modified
    if (this->dynamics_time_step != this->world->GetPhysicsEngine()->GetStepTime())
      this->initialize_controllers();

    // update joints
    for (unsigned int i = 0; i < this->joints.size(); i++)
    {
      // update setpoints and sensor values
      double p_gain = this->p_gain_constant*dcm.joint_p_gain[i];
      double i_gain = this->i_gain_constant*dcm.joint_i_gain[i];
      double d_gain = this->d_gain_constant*dcm.joint_d_gain[i];

      double force_setpoint = dcm.joint_force[i];
      double position_setpoint = dcm.joint_position[i];
      double velocity_setpoint = dcm.joint_velocity[i];
      double position_actual = this->joints[i]->GetAngle(0).Radian();
      double velocity_actual = this->joints[i]->GetVelocity(0);

      // update feedforward force
      double force_command = force_setpoint;

      // update position controller
      struct pid *position_pid = &joint_position_pids[i];
      pid_set_setpoint(position_pid, position_setpoint);
      pid_set_gains(position_pid,
        p_gain,
        i_gain,
        0
      );
      force_command += pid_update(position_pid, position_actual);

      // update velocity controller
      struct filter *velocity_filter = &joint_velocity_filters[i];
      double velocity_estimate = filter_update(velocity_filter, velocity_actual);
      force_command += d_gain*(velocity_setpoint - velocity_estimate);
       
      // update joint force 
      double max_force = this->joints[i]->GetEffortLimit(0);
      force_command = math::clamp(force_command, -max_force, max_force);
      this->joints[i]->SetForce(0, force_command);

      // update joint sensors
      dcm.joint_force_sensor[i] = force_command;
      dcm.joint_position_sensor[i] = position_actual;
      dcm.joint_velocity_sensor[i] = velocity_actual;

    //gzerr << "force command " << force_command << "\n";
    }
  }

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(gz_comms_manager)
}
