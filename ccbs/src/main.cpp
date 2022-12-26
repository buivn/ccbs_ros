#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include <ccbs/ccbs.h>



// int main(int argc, const char *argv[])
int main(int argc, char **argv)
{
  const std::string map_file = "/home/dzungbui/ros_noetic/src/ccbs/src/graph.xml";

  ros::init(argc, argv, "ccbs_server");

  ccbs::MAMP mamp(map_file);
  
  // ros::NodeHandle n;
  ROS_INFO("Ready to return the paths.");
  ros::spin();

  return 0;
}
