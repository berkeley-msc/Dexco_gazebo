#ifndef DEXCO_VARIABLE_STIFFNESS_PLUGIN_H
#define DEXCO_VARIABLE_STIFFNESS_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float64.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <sensor_msgs/JointState.h>
#include <thread>
#include <map>
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>

namespace gazebo
{
  class DexcoVariableStiffnessPlugin : public ModelPlugin
  {
  public:
    DexcoVariableStiffnessPlugin() : ModelPlugin() {
    }

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      // Store the model pointer for convenience.
      this->model = _model;

      // Start ROS node
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "dexco_variable_stiffness_plugin", ros::init_options::NoSigintHandler);
      }

      this->rosNode.reset(new ros::NodeHandle("dexco_variable_stiffness_plugin"));

      // Subscribe to desired joint states topic
      ros::SubscribeOptions so_joint_states = ros::SubscribeOptions::create<sensor_msgs::JointState>(
          "/custom_joint_states",
          1,
          boost::bind(&DexcoVariableStiffnessPlugin::OnJointStateMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);
      this->jointStateSub = this->rosNode->subscribe(so_joint_states);

      // // Subscribe to current joint states topic
      // ros::SubscribeOptions so_joint_states = ros::SubscribeOptions::create<sensor_msgs::JointState>(
      //     "/joint_states",
      //     1,
      //     boost::bind(&DexcoVariableStiffnessPlugin::OnJointStateMsg, this, _1),
      //     ros::VoidPtr(), &this->rosQueue);
      // this->jointStateSub = this->rosNode->subscribe(so_joint_states);

      // Initialize the joints and ROS publishers for each joint stiffness
      for (auto joint : this->model->GetJoints())
      {
        // this->joints.push_back(joint);
        std::string joint_name = joint->GetName();
        this->joints[joint_name] = joint;
        // ros::SubscribeOptions so_stiffness = ros::SubscribeOptions::create<std_msgs::Float64>(
        //     "/" + this->model->GetName() + "/" + joint_name + "_stiffness",
        //     1,
        //     boost::bind(&DexcoVariableStiffnessPlugin::OnStiffnessMsg, this, _1, joint_name),
        //     ros::VoidPtr(), &this->rosQueue);
        // this->stiffnessSubs[joint_name] = this->rosNode->subscribe(so_stiffness);
        this->desiredStiffness[joint_name] = 0.0;
        this->externalJointPositions[joint_name] = 0.0;
      }

      // Spin up the queue helper thread.
      this->rosQueueThread = std::thread(std::bind(&DexcoVariableStiffnessPlugin::QueueThread, this));
    }

    // Set stiffness based on ROS message
    public: void OnStiffnessMsg(const std_msgs::Float64ConstPtr &_msg, const std::string &joint_name)
    {
      this->desiredStiffness[joint_name] = _msg->data;
    }

    // Process joint state messages
    public: void OnJointStateMsg(const sensor_msgs::JointState::ConstPtr &_msg)
    {
      for (size_t i = 0; i < _msg->name.size(); ++i)
      {
        this->externalJointPositions[_msg->name[i]] = _msg->position[i];
      }
    }

    // Update stiffness and apply effort
    private: void UpdateStiffnessAndEffort()
    {
      static int count = 0;

      double currentPositionq_LJ0 = this->joints["LJ0"]->Position(0);
      double currentPositionq_LJ1 = this->joints["LJ1"]->Position(0);
      double currentPositionq_LJ2 = this->joints["LJ2"]->Position(0);
      double currentPositionq_LJ3 = this->joints["LJ3"]->Position(0);
      double currentPositionq_RJ0 = this->joints["RJ0"]->Position(0);
      double currentPositionq_RJ1 = this->joints["RJ1"]->Position(0);
      double currentPositionq_RJ2 = this->joints["RJ2"]->Position(0);
      double currentPositionq_RJ3 = this->joints["RJ3"]->Position(0);
      
      auto currentPositionl_LJ0 = currentPositionq_LJ0;
      auto currentPositionl_LJ1 = 0.0135*currentPositionq_LJ1+0.015*currentPositionq_LJ2;
      auto currentPositionl_LJ2 = -0.0135*currentPositionq_LJ1+0.015*currentPositionq_LJ2;
      auto currentPositionl_LJ3 = 0.015*currentPositionq_LJ3;
      auto currentPositionl_RJ0 = currentPositionq_RJ0;
      auto currentPositionl_RJ1 = -0.0135*currentPositionq_RJ1+0.015*currentPositionq_RJ2;
      auto currentPositionl_RJ2 = 0.0135*currentPositionq_RJ1+0.015*currentPositionq_RJ2;
      auto currentPositionl_RJ3 = 0.015*currentPositionq_RJ3;

      double q0_LJ0 = this->externalJointPositions["LJ0"];
      double q0_LJ1 = this->externalJointPositions["LJ1"];
      double q0_LJ2 = this->externalJointPositions["LJ2"];
      double q0_LJ3 = this->externalJointPositions["LJ3"];
      double q0_RJ0 = this->externalJointPositions["RJ0"];
      double q0_RJ1 = this->externalJointPositions["RJ1"];
      double q0_RJ2 = this->externalJointPositions["RJ2"];
      double q0_RJ3 = this->externalJointPositions["RJ3"];

      auto l0_LJ0 = q0_LJ0;
      auto l0_LJ1 = 0.0135*q0_LJ1 + 0.015*q0_LJ2;
      auto l0_LJ2 = -0.0135*q0_LJ1 + 0.015*q0_LJ2;
      auto l0_LJ3 = 0.015*q0_LJ3;
      auto l0_RJ0 = -l0_LJ0;
      auto l0_RJ1 = -0.0135*q0_RJ1 + 0.015*q0_RJ2;
      auto l0_RJ2 = 0.0135*q0_RJ1 + 0.015*q0_RJ2;
      auto l0_RJ3 = 0.015*q0_RJ3;

      auto delta_l_LJ0 = currentPositionl_LJ0 - l0_LJ0;
      auto delta_l_LJ1 = currentPositionl_LJ1 - l0_LJ1;
      auto delta_l_LJ2 = currentPositionl_LJ2 - l0_LJ2;
      auto delta_l_LJ3 = currentPositionl_LJ3 - l0_LJ3;
      auto delta_l_RJ0 = currentPositionl_RJ0 - l0_RJ0;
      auto delta_l_RJ1 = currentPositionl_RJ1 - l0_RJ1;
      auto delta_l_RJ2 = currentPositionl_RJ2 - l0_RJ2;
      auto delta_l_RJ3 = currentPositionl_RJ3 - l0_RJ3;


      auto effortl_LJ0 = -1600*delta_l_LJ0;
      auto effortl_LJ1 = ComputeEffortNonlinear(-l0_LJ1,-delta_l_LJ1);
      auto effortl_LJ2 = ComputeEffortNonlinear(-l0_LJ2,-delta_l_LJ2);
      auto effortl_LJ3 = ComputeEffortNonlinear(-l0_LJ3,-delta_l_LJ3);
      auto effortl_RJ0 = -1600*delta_l_RJ0;
      auto effortl_RJ1 = ComputeEffortNonlinear(-l0_RJ1,-delta_l_RJ1);
      auto effortl_RJ2 = ComputeEffortNonlinear(-l0_RJ2,-delta_l_RJ2);
      auto effortl_RJ3 = ComputeEffortNonlinear(-l0_RJ3,-delta_l_RJ3);

      auto effortq_LJ0 = effortl_LJ0;
      auto effortq_LJ1 = 0.0135*2*effortl_LJ1-0.0135*2*effortl_LJ2;
      auto effortq_LJ2 = 0.015*effortl_LJ1+0.015*effortl_LJ2;
      auto effortq_LJ3 = 0.015*effortl_LJ3;
      auto effortq_RJ0 = effortl_RJ0;
      auto effortq_RJ1 = -0.0135*2*effortl_RJ1+0.0135*2*effortl_RJ2;
      auto effortq_RJ2 = 0.015*effortl_RJ1+0.015*effortl_RJ2;
      auto effortq_RJ3 = 0.015*effortl_RJ3;

      this->joints["LJ0"]->SetForce(0, effortq_LJ0);
      this->joints["LJ1"]->SetForce(0, effortq_LJ1);
      this->joints["LJ2"]->SetForce(0, effortq_LJ2);
      this->joints["LJ3"]->SetForce(0, effortq_LJ3);
      this->joints["RJ1"]->SetForce(0, effortq_RJ1);
      this->joints["RJ2"]->SetForce(0, effortq_RJ2);
      this->joints["RJ3"]->SetForce(0, effortq_RJ3);
      this->joints["RJ0"]->SetForce(0, effortq_RJ0);

      // if ((++count) % 1000 == 0){
      //   std::cout << std::fixed << std::setprecision(4) << "current q_LJ1: " << currentPositionq_LJ1 << "\tcurrent l1 " << currentPositionl_LJ1 << "\tcurrent l2 " << currentPositionl_LJ2 << "\tl0_LJ1 " << l0_LJ1 << "\tl0_LJ2 " << l0_LJ2 << "\tdelta_l_LJ1 " << delta_l_LJ2 << "\tdelta_l_LJ2 " << delta_l_LJ2  << "\teffortl_LJ1 " << effortl_LJ1 << "\teffortl_LJ2 " << effortl_LJ2 << "\tq0_LJ1 " << q0_LJ1 <<std::endl;
      // }
    }

    // Compute stiffness based on position difference
    private: double ComputeEffortNonlinear(double l0, double delta_l)
    {
      auto y = delta_l; auto x = l0 + 0.02;
      auto effort = (17.14-560*x-1621*y+324757*x*y-160958*y*y+4360743*x*y*y+26265709*y*y*y);
      return effort;
    }
    // Compute stiffness based on stiffness matrix
    private: double ComputeStiffnessK(double positionDifference)
    {
      // Example stiffness function: k = 100 * (1 + sin(positionDifference))
      return 1;
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
    // std::vector<physics::JointPtr> joints;
    std::unique_ptr<ros::NodeHandle> rosNode;
    // std::map<std::string, ros::Subscriber> stiffnessSubs;
    ros::Subscriber jointStateSub;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;
    std::map<std::string, double> desiredStiffness;
    std::map<std::string, double> externalJointPositions;
    std::map<std::string, physics::JointPtr> joints;
  };

  GZ_REGISTER_MODEL_PLUGIN(DexcoVariableStiffnessPlugin)
}

#endif
