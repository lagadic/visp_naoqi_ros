#include <ros/ros.h>

#include <visp_naoqi/vpNaoqiRobot.h>
#include <sensor_msgs/JointState.h>
#include "visp_naoqi_robot.h"

int main( int argc, char** argv )
{
  ros::init( argc, argv, "visp_naoqi_ros" );

  ros::NodeHandle n(std::string("~"));

  VispNaoqiRobot *node = new VispNaoqiRobot(n);

  node->spin();

  delete node;

  return 1;
}



