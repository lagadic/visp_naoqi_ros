#include <iostream>
#include <vector>
#include <algorithm>

#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Transform.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>

#include "visp_naoqi_robot.h"

VispNaoqiRobot::VispNaoqiRobot(ros::NodeHandle &nh)
{

  TFname= "";
  // read in config options
  n = nh;

  n.param( "frequency", freq, 100);
  n.param<std::string>("VelocityCmdTopicName", cmdVelTopicName, "cmd_vel");
  n.param<std::string>("JointStateTopicName", jointStateTopicName, "joint_state");
  n.param<std::string>("Ip", ip, "198.18.0.1");
  n.param("Port", port, 9559);
  n.param("PublishTFdata", pub_tf_data, true);
  n.param<std::string>("TFdataFile", TFdataFile, "");
  n.param<std::string>("TFname", TFname, "");
  n.param("UseMCRobotState", use_mc_robot_state, false);
  n.param("PublishRobotState", pub_robot_state, true);

  // Initialize subscriber and publisher
  jointVelSub = n.subscribe( cmdVelTopicName, 1, (boost::function < void(const sensor_msgs::JointStateConstPtr&)>) boost::bind( &VispNaoqiRobot::setJointVel, this, _1 ));
  //  jointStatePub = n.advertise<sensor_msgs::JointState>(jointStateTopicName, 10);
  //  jointMCStatePub = n.advertise<sensor_msgs::JointState>(jointStateTopicName, 10);


  ROS_INFO("Launch NaoqiRobotros node");
  romeo = new vpNaoqiRobot;
  romeo->setRobotIp(ip);
  romeo->open();

  // Initialize names msg JointState
  jointBodyNames = romeo->getProxy()->getBodyNames("Body");
  if (pub_robot_state)
  {
    if (!use_mc_robot_state)
    {
      jointStateMsg.name = jointBodyNames;
      jointStatePub = n.advertise<sensor_msgs::JointState>(jointStateTopicName, 10);
    }
    else
    {
      jointMCStateMsg.joint_name = jointBodyNames;
      jointMCStatePub = n.advertise<mc_robot_msgs::MCRobotState>(jointStateTopicName, 10);
    }

    std::stringstream ss;
    std::copy(jointBodyNames.begin(), jointBodyNames.end()-1, std::ostream_iterator<std::string>(ss,","));
    std::copy(jointBodyNames.end()-1, jointBodyNames.end(), std::ostream_iterator<std::string>(ss));
    ROS_INFO("Romeo joints found: %s",ss.str().c_str());

    velState.resize(jointBodyNames.size());
    posState.resize(jointBodyNames.size());
  }
  if (pub_tf_data)
  {
    std::string TFname= "";
    n.param<std::string>("TFname", TFname, "");
    vpXmlParserHomogeneousMatrix pm; // Create a XML parser

    vpHomogeneousMatrix M;

    if (pm.parse(M, TFdataFile, TFname) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      std::cout << "Cannot found the Homogeneous matrix named " << TFname<< "." << std::endl;
    }
    else
      std::cout << "Homogeneous matrix " << TFname <<": " << std::endl << M << std::endl;

    pose_hand_target = visp_bridge::toGeometryMsgsPose(M); //convert
  }

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
    if (pub_robot_state)
      this->publish();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void VispNaoqiRobot::publish()
{
  try
  {
    posState = romeo->getProxy()->getAngles(jointBodyNames,true);
    //    romeo->getJointVelocity(jointBodyNames,velState);
    //    std::vector<float>::iterator it = velState.begin();


    //    std::vector<float> zeroVec (4, 0.0);
    //    velState.insert (it+4,zeroVec.begin(),zeroVec.end());


  }
  catch(const AL::ALError& e)
  {
    ROS_ERROR("Could not get joint data from Romeo.\n\tTrace: %s",e.what());
    return;
  }

  //  std::cout <<"Body:" << jointBodyNames <<"size:" << jointBodyNames.size() << std::endl;
  //  std::cout <<"Pose:" << posState <<"size:" << posState.size() << std::endl;
  //  std::cout <<"Vel:" << velState <<"size:" << velState.size() << std::endl;

  if (posState.size() != velState.size()){
    ROS_ERROR("The vector of the joint state name and of the velocity have a different size.");
    return;
  }

  if (!use_mc_robot_state)
  {
    jointStateMsg.header.stamp = ros::Time::now();
    jointStateMsg.header.frame_id = "base_link";

    jointStateMsg.position.resize(posState.size());
    //  jointStateMsg.velocity.resize(posState.size());

    for(unsigned int i = 0; i<posState.size(); ++i)
    {
      jointStateMsg.position[i] = posState[i];
      //    jointStateMsg.velocity[i] = velState[i];
    }

    jointStatePub.publish(jointStateMsg);
  }
  else
  {

    jointMCStateMsg.header.stamp  = ros::Time::now();
    jointMCStateMsg.header.frame_id =  "base_link";
    jointMCStateMsg.joint_position.resize(posState.size());

    for(unsigned int i = 0; i<posState.size(); ++i)
    {
      jointMCStateMsg.joint_position[i] = posState[i];
    }

    jointMCStatePub.publish(jointMCStateMsg);

  }

  if (pub_tf_data)
  {
    //TODO: convert in a better way
    tf::Transform transform;
    static tf::TransformBroadcaster br;
    transform.setOrigin( tf::Vector3(pose_hand_target.position.x, pose_hand_target.position.y, pose_hand_target.position.z) );
    tf::Quaternion q;
    q.setX(pose_hand_target.orientation.x);
    q.setY(pose_hand_target.orientation.y);
    q.setZ(pose_hand_target.orientation.z);
    q.setW(pose_hand_target.orientation.w);
    transform.setRotation(q);
    //br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "LWristPitch", "rhand_target"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "object"));
    std::cout << "Publishing tf" << std::endl;
  }

  return;
}


void VispNaoqiRobot::setJointVel(const sensor_msgs::JointStateConstPtr &msg)
{
  ros::Time veltime = ros::Time::now();


  if (msg->velocity.size() != msg->name.size()) {
    ROS_ERROR("The vector of the joint name and of the velocity have a different size.");
    return;
  }

  //ros::Time time = ros::Time::now();
  romeo->setVelocity(msg->name, msg->velocity);
  //std::cout << "Loop Time: " << ros::Time::now()-time << std::endl;

  ROS_INFO("Applying vel at %f s. ID: %f:",veltime.toSec(),msg->header.seq );


}

