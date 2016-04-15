#include <ros/ros.h>
#include <visp_naoqi/vpNaoqiRobot.h>
#include <sensor_msgs/JointState.h>

class VispNaoqiRobot
{
public:

  VispNaoqiRobot(ros::NodeHandle &nh);
  ~VispNaoqiRobot();
  void publish();
  void spin();
  void setJointVel(const sensor_msgs::JointStateConstPtr &msg);

protected:

  // Robot
  vpNaoqiRobot * romeo;
  int port;
  std::string ip;
  std::vector <std::string> jointBodyNames;
  // ROS
  ros::NodeHandle n;
  ros::Time veltime;
  std::string cmdVelTopicName;  // default to /cmd_vel
  std::string jointStateTopicName;
  int freq;
  ros::Subscriber jointVelSub;
  ros::Publisher jointStatePub;
  // Messages
  sensor_msgs::JointState jointStateMsg;
  std::vector<float> posState;
  std::vector<float> velState;


};
