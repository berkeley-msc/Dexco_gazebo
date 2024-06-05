#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

class TestJointStateToCommand
{
public:
  TestJointStateToCommand()
  {
    // Subscribe to /joint_states
    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &TestJointStateToCommand::jointStateCallback, this);
    
    // Publisher for the /test/joint1_position_controller/command
    joint1_command_pub_ = nh_.advertise<std_msgs::Float64>("/test/joint1_position_controller/command", 10);
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    // Find the index of joint1 in the joint states
    auto it = std::find(msg->name.begin(), msg->name.end(), "joint1");
    if (it != msg->name.end())
    {
      int index = std::distance(msg->name.begin(), it);
      
      // Get the position of joint1
      if (index < msg->position.size())
      {
        std_msgs::Float64 position_msg;
        position_msg.data = msg->position[index];
        joint1_command_pub_.publish(position_msg);
      }
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber joint_state_sub_;
  ros::Publisher joint1_command_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_joint_state_to_command");
  TestJointStateToCommand node;
  ros::spin();
  return 0;
}
