#include <iostream>
#include <vector>

#include "visp_naoqi_robot.h"
#include <algorithm>


VispNaoqiRobot::VispNaoqiRobot(ros::NodeHandle &nh)
{
  // read in config options
  n = nh;

  n.param( "frequency", freq,100);
  n.param<std::string>("VelocityCmdTopicName", cmdVelTopicName, "cmd_vel");
  n.param<std::string>("JointStateTopicName", jointStateTopicName, "joint_state");
  n.param<std::string>("Ip", ip, "198.18.0.1");
  n.param("Port", port, 9559);

  // Initialize subscriber and publisher
  jointVelSub = n.subscribe( cmdVelTopicName, 1, (boost::function < void(const sensor_msgs::JointStateConstPtr&)>) boost::bind( &VispNaoqiRobot::setJointVel, this, _1 ));
  jointStatePub = n.advertise<sensor_msgs::JointState>(jointStateTopicName, 10);

  ROS_INFO("Launch NaoqiRobotros node");
  romeo = new vpNaoqiRobot;
  romeo->setRobotIp(ip);
  romeo->open();

  // Initialize names msg JointState
  jointBodyNames = romeo->getProxy()->getBodyNames("Body");
  jointStateMsg.name = jointBodyNames;

  std::stringstream ss;
  std::copy(jointBodyNames.begin(), jointBodyNames.end()-1, std::ostream_iterator<std::string>(ss,","));
  std::copy(jointBodyNames.end()-1, jointBodyNames.end(), std::ostream_iterator<std::string>(ss));
  ROS_INFO("Romeo joints found: %s",ss.str().c_str());

}

VispNaoqiRobot::~VispNaoqiRobot(){

  if (romeo){
    romeo->stop(jointBodyNames);
    delete romeo;
    romeo = NULL;
  }
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

void VispNaoqiRobot::publish()
{
  try
  {
    posState = romeo->getProxy()->getAngles("Body",true);
    romeo->getJointVelocity(jointBodyNames,velState);
  }
  catch(const AL::ALError& e)
  {
    ROS_ERROR("Could not get joint data from Romeo.\n\tTrace: %s",e.what());
    return;
  }

  if (posState.size() != velState.size()){
    ROS_ERROR("The vector of the joint state name and of the velocity have a different size.");
    return;
  }

  jointStateMsg.header.stamp = ros::Time::now();
  jointStateMsg.header.frame_id = "base_link";


    jointStateMsg.position.resize(posState.size());
    for(unsigned i = 0; i<posState.size(); ++i)
    {
       jointStateMsg.position[i] = posState[i];
       jointStateMsg.velocity[i] = velState[i];
    }

  jointStatePub.publish(jointStateMsg);
}




void VispNaoqiRobot::setJointVel(const sensor_msgs::JointStateConstPtr &msg)
{
  //ros::Time veltime = ros::Time::now();


  if (msg->velocity.size() != msg->name.size()) {
    ROS_ERROR("The vector of the joint name and of the velocity have a different size.");
    return;
  }

  romeo->setVelocity(msg->name, msg->velocity);
  ROS_INFO("Applying vel at %f s:",veltime.toSec());


}

