#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpXmlParserHomogeneousMatrix.h>

#include <visp_bridge/3dpose.h>



int main( int argc, char** argv )
{
  ros::init( argc, argv, "visp_naoqi_ros" );

  ros::NodeHandle n(std::string("~"));

  vpXmlParserHomogeneousMatrix pm; // Create a XML parser

  //  char filename_[FILENAME_MAX];
  //  sprintf(filename_, "%s", VISP_NAOQI_GENERAL_M_FILE);

  vpHomogeneousMatrix M;

  if (pm.parse(M, "/udd/gclaudio/romeo/cpp/workspace/romeo_tk/data/transformation.xml", "qrcode_M_e_LArm") != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
    std::cout << "Cannot found the Homogeneous matrix named " << transform_name<< "." << std::endl;
    exit(0);
  }
  else
    std::cout << "Homogeneous matrix " << transform_name <<": " << std::endl << oMe_hand << std::endl;

  geometry_msgs::Pose msg_pose;
  msg_pose.pose = visp_bridge::toGeometryMsgsPose(M); //convert

  visp_bridge::toGeometryMsgsTransform()




  return 1;
}



