#ifndef VARIABLE_STIFFNESS_PLUGIN_H
#define VARIABLE_STIFFNESS_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float64.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <sensor_msgs/JointState.h>
#include <thread>
#include <algorithm>

namespace gazebo
{
  class VariableStiffnessPlugin : public ModelPlugin
  {
  public:
    VariableStiffnessPlugin() : ModelPlugin() {
    }

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store the model pointer for convenience.
      this->model = _model;
      this->joint = _model->GetJoint("joint1");

      // Start ROS node
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "variable_stiffness_plugin", ros::init_options::NoSigintHandler);
      }

      this->rosNode.reset(new ros::NodeHandle("variable_stiffness_plugin"));

      // Subscribe to joint stiffness topic
      ros::SubscribeOptions so_stiffness = ros::SubscribeOptions::create<std_msgs::Float64>(
          "/" + this->model->GetName() + "/joint1_stiffness",
          1,
          boost::bind(&VariableStiffnessPlugin::OnStiffnessMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
      this->stiffnessSub = this->rosNode->subscribe(so_stiffness);

      // Subscribe to joint states topic
      ros::SubscribeOptions so_joint_states = ros::SubscribeOptions::create<sensor_msgs::JointState>(
          "/joint_states",
          1,
          boost::bind(&VariableStiffnessPlugin::OnJointStateMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
      this->jointStateSub = this->rosNode->subscribe(so_joint_states);

      // Spin up the queue helper thread.
      this->rosQueueThread = std::thread(std::bind(&VariableStiffnessPlugin::QueueThread, this));
    }

    // Set stiffness based on ROS message
    public: void OnStiffnessMsg(const std_msgs::Float64ConstPtr &_msg)
    {
      this->desiredStiffness = _msg->data;
    }

    // Process joint state messages
    public: void OnJointStateMsg(const sensor_msgs::JointState::ConstPtr &_msg)
    {
      auto it = std::find(_msg->name.begin(), _msg->name.end(), "joint1");
      if (it != _msg->name.end())
      {
        int index = std::distance(_msg->name.begin(), it);
        if (index < _msg->position.size())
        {
          this->externalJointPosition = _msg->position[index];
        }
      }
    }

    // Update stiffness and apply effort
    private: void UpdateStiffnessAndEffort()
    {
      double currentPosition = this->joint->Position(0);
      double stiffness = ComputeStiffness(this->externalJointPosition);
      double effort = stiffness * (this->externalJointPosition - currentPosition);
      this->joint->SetForce(0, effort);
    }

    // Compute stiffness based on position difference
    private: double ComputeStiffness(double positionDifference)
    {
      // Example stiffness function: k = 100 * (1 + sin(positionDifference))
      return 100.0;
    }

    // ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
        this->UpdateStiffnessAndEffort();
      }
    }

  private:
    physics::ModelPtr model;
    physics::JointPtr joint;
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Subscriber stiffnessSub;
    ros::Subscriber jointStateSub;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;
    double desiredStiffness = 0.0;
    double externalJointPosition = 0.0;
  };

  GZ_REGISTER_MODEL_PLUGIN(VariableStiffnessPlugin)
}

#endif
