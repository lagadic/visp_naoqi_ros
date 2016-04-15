#include <ros/ros.h>
#include <visp_naoqi/vpNaoqiRobot.h>
class VispNaoqiRobot
{
public:

  VispNaoqiRobot(ros::NodeHandle nh);
  virtual ~RosViper850Node();
  void publish();
  void spin();

protected:

  // Robot
  vpNaoqiRobot * romeo;
  unsigned int port;
  std::string ip;
  // ROS
  ros::NodeHandle n;
  ros::Time veltime;
  std::string cmdVelTopicName;  // default to /cmd_vel
  std::string jointStateTopicName;
  unsigned int freq;
  ros::Subscriber joint_vel_sub;
  ros::Publisher joint_state_pub;

};
