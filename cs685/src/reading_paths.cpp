#include <ros/ros.h>
#include <cs685/GetMultiPaths.h>
#include <nav_msgs/Path.h>
#include <cs685/tinyxml2.h>
#include <geometry_msgs/PoseStamped.h>

std::vector<nav_msgs::Path> paths;

bool get_paths(cs685::GetMultiPaths::Request  &req,
              cs685::GetMultiPaths::Response &res)
  {
    for (int i = 0; i < paths.size(); i++) {
      res.paths.push_back(paths[i]);
    }  
    // ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    return true;
  }


int main (int argc, char **argv)
{
  
  nav_msgs::Path path_n;
  tinyxml2::XMLElement *root = 0, *agent = 0, *path=0, *robot=0, *goal=0;
  tinyxml2::XMLDocument doc;

  // Load XML File
  if (doc.LoadFile("/home/dzungbui/Continuous-CBS/graph/graph_task_log.xml") 
            != tinyxml2::XMLError::XML_SUCCESS)
  {
      std::cout << "Error opening XML file!" << std::endl;
      return false;
  }

  // Get ROOT element
  root = doc.FirstChildElement("root");
  root = root->FirstChildElement("log");
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "map";
  
  for (robot = root->FirstChildElement("agent"); robot; robot = robot->NextSiblingElement()) {
    path = robot->FirstChildElement("path");
    path_n.poses.clear();

    if (!path)
    {
      std::cout << "Error! No '" << "path" << "' tag found in XML file!" << std::endl;
      return false;
    }
    double time_flow =0.0;
    for (agent = path->FirstChildElement(); agent; agent = agent->NextSiblingElement())
    {
      p.pose.position.x = agent->DoubleAttribute("start_i"); 
      p.pose.position.y = agent->DoubleAttribute("start_j");

      p.header.stamp = ros::Time(time_flow);
      time_flow += agent->DoubleAttribute("duration");
      goal = agent;
      path_n.poses.push_back(p);
    }

    p.pose.position.x = goal->DoubleAttribute("goal_i"); 
    p.pose.position.y = goal->DoubleAttribute("goal_j");
    p.header.stamp = ros::Time(time_flow);
    path_n.poses.push_back(p);

    paths.push_back(path_n);
  }
  
  ros::init(argc, argv, "reading_paths_node");
  ros::NodeHandle n;
  ros::ServiceServer paths_service = n.advertiseService("robot_paths", get_paths);
  ROS_INFO("Ready to return the paths.");
  ros::spin();
  return 0;
}
