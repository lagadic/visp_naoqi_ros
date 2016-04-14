#include <ros/ros.h>

#include "visp_naoqi_robot.h"


int main( int argc, char** argv )
{
  ros::init( argc, argv, "visp_naoqi_ros" );

  VispNaoqiRobot _robot;

  ros::AsyncSpinner spinner( 1 );
  spinner.start();

  int freq = 10; // in Hz
  ros::NodeHandle nh;
  nh.param<int>( "freq", freq, freq );
  ros::Rate r( freq );

  while( ros::ok() )
  {
    ros::Time t = ros::Time::now();

    _robot.readJoints();

    _robot.writeJoints();

    r.sleep();
  }

  spinner.stop();

  return 1;
}
