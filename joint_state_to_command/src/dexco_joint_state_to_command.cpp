#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <map>
#include <vector>
#include <iostream>

class DexcoJointStateToCommand
{
public:
  DexcoJointStateToCommand()
  {
    // Subscribe to /joint_states
    joint_state_sub_ = nh_.subscribe("/dexco/joint_states", 10, &DexcoJointStateToCommand::jointStateCallback, this);

    // Initialize publishers for RJ0 joint position controller
    jointq5_command_pub_ = nh_.advertise<std_msgs::Float64>("/dexco/jointq5_position_controller/command", 10);
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    // Find the index of q1 in the joint states
    auto it = std::find(msg->name.begin(), msg->name.end(), "LJ0");
    if (it != msg->name.end())
    {
      int index = std::distance(msg->name.begin(), it);
      
      // Get the position of joint1
      if (index < msg->position.size())
      {
        std_msgs::Float64 position_msg;
        position_msg.data = -msg->position[index];
        jointq5_command_pub_.publish(position_msg);
      }
    }
    // size_t q1 = 0;
    // // Find the index of q5 in the joint states
    // for (size_t i = 0; i < msg->name.size(); ++i)
    // {
    //   const std::string& joint_name = msg->name[i];
    //   // Check if the joint is one of the controlled joints
    //   if (joint_command_pubs_.find(joint_name) != joint_command_pubs_.end() && joint_name=="LJ0")
    //   {
    //     q1 = i; 
    //     printf("here is LJ0");
    //   }
    // }
    // jointq5_command_pub_.publish(-msg->position[q1]);
    // if (joint_name=="RJ0") {
    //   joint_command_pubs_[joint_name].publish(-msg->position[0]);
    // } else {
    //   std_msgs::Float64 position_msg;
    //   position_msg.data = msg->position[i];
    //   joint_command_pubs_[joint_name].publish(position_msg);
    // }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher jointq5_command_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dexco_joint_state_to_command");
  DexcoJointStateToCommand node;
  ros::spin();
  return 0;
}
