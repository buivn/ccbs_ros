#ifndef CCBS_H
#define CCBS_H
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include <ccbs/map.h>
#include <ccbs/task.h>
#include <ccbs/cbs.h>
#include <ccbs/structs.h>
#include <ccbs/xml_logger.h>
#include <cs685/GetMultiPaths.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

namespace ccbs {

  class MAMP
  {
    public:
      MAMP(const std::string map_file);
      ~MAMP(){}
      // bool run();
      bool find_paths(cs685::GetMultiPaths::Request  &req,
                cs685::GetMultiPaths::Response &res);
    private:
      Config config;
      Map map;
      Task task;
      CBS cbs;
      Solution solution;
      ros::ServiceServer make_paths_srv_;

};

}
#endif // CCBS_H
