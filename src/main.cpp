#include <ros/ros.h>

#include <visp_naoqi/vpNaoqiRobot.h>
#include <sensor_msgs/JointState.h>
#include "visp_naoqi_robot.h"

int main( int argc, char** argv )
{
  ros::init( argc, argv, "visp_naoqi_ros" );

  VispNaoqiRobot _robot;

  ros::NodeHandle n(std::string("~"));

  VispNaoqiRobot *node = new VispNaoqiRobot(n);

  if( node->setup() != 0 )
  {
    printf( "VispNaoqiRobot setup failed... \n" );
    return -1;
  }

  node->spin();

  delete node;

  // -----------------------------------------------
  
  // Topic names
  std::string cmdVelTopicName;  



  ros::AsyncSpinner spinner( 1 );
  spinner.start();

  int freq = 100; // in Hz

// Load parameters
  nh.param<int>( "freq", freq, freq );
  nh.param<std::string>("cmd_vel_topic_name", cmdVelTopicName, "cmd_vel");

  
  ros::Rate r( freq );
  
  // Initialize subscriber:
  ros::Subscriber jointvel_sub = n.subscribe( cmdVelTopicName, 1, (boost::function < void(const sensor_msgs::JointStateConstPtr&)>) boost::bind( &setJointVel, this, _1 ));

  while( ros::ok() )
  {
    ros::Time t = ros::Time::now();

//    _robot.readJoints();

//    _robot.writeJoints();

    r.sleep();
  }


  spinner.stop();
// -----------------------------------------------
  return 1;
}



