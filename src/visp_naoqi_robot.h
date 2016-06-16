#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Transform.h>
#include <mc_robot_msgs/MCRobotState.h>

#include <visp_bridge/3dpose.h>

#include <visp_naoqi/vpNaoqiRobot.h>


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
  std::string cmdVelTopicName;
  std::string jointStateTopicName;
  std::string TFdataFile;
  std::string TFname;
  geometry_msgs::Pose pose_hand_target;
  int freq;
  bool pub_tf_data;
  bool use_mc_robot_state;
  bool pub_robot_state;
  ros::Subscriber jointVelSub;
  ros::Publisher jointStatePub;
  ros::Publisher jointMCStatePub;
  // Messages
  sensor_msgs::JointState jointStateMsg;
  mc_robot_msgs::MCRobotState jointMCStateMsg;
  std::vector<float> posState;
  std::vector<float> velState;


};
