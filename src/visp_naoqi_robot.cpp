#include <iostream>
#include <vector>

#include "visp_naoqi_robot.h"



VispNaoqiRobot::VispNaoqiRobot(ros::NodeHandle nh)
{
  // read in config options
  n = nh;

  n.param<int>( "freq", freq, freq );
  n.param<std::string>("cmd_vel_topic_name", cmdVelTopicName, "cmd_vel");
  n.param<std::string>("joint_state_topic_name", jointStateTopicName, "joint_state");
  n.param<std::string>("ip", ip, 198.18.0.1);
  n.param<int>("port", port, 9559);

  // Initialize subscriber and publisher
  joint_vel_sub = n.subscribe( cmdVelTopicName, 1, (boost::function < void(const sensor_msgs::JointStateConstPtr&)>) boost::bind( &VispNaoqiRobot::setJointVel, this, _1 ));
  joint_state_pub = n.advertise<sensor_msgs::JointState>(jointStateTopicName, 10);

  ROS_INFO("Launch NaoqiRobotros node");
  romeo = new vpNaoqiRobot;
  romeo->setRobotIp(ip);
  romeo->open();




}

void VispNaoqiRobot::readJoints()
{


  ROS_INFO_STREAM("*****read*******");
  ROS_DEBUG("I just computed read joints");
}

void VispNaoqiRobot::writeJoints()
{

  ROS_INFO_STREAM("*****write*******");
  ROS_DEBUG("I just computed write joints");
}

void VispNaoqiRobot::spin()
{
  ros::Rate loop_rate(100);
  while(ros::ok()){
    this->publish();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void VispNaoqiRobot::setJointVel(const sensor_msgs::JointStateConstPtr &msg)
{
  ros::Time veltime = ros::Time::now();

  if (msg->velocity.size() != msg->name.size()) {
    ROS_ERROR("The vector of the joint name and of the velocity have a different size.");
    return;
  }
  vpColVector qdot(6); // Vel in rad/s for each joint



  for(unsigned int i=0; i< msg->velocity.size(); i++)
    qdot[i] = msg->velocity[i];

  ROS_INFO("Viper850 new joint vel at %f s: [%0.2f %0.2f %0.2f %0.2f %0.2f %0.2f] rad/s",
           veltime.toSec(),
           qdot[0], qdot[1], qdot[2], qdot[3], qdot[4], qdot[5]);

  robot->setVelocity(vpRobot::ARTICULAR_FRAME, qdot);
}

