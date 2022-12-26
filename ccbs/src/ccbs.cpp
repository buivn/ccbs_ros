#include <iostream>
#include <fstream>
#include <string>
#include <ccbs/ccbs.h>

namespace ccbs {

  MAMP::MAMP(const std::string map_file) {
    // ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    map.initialize(config.agent_size, config.connectdness);
    map.get_map(map_file.c_str()); // get a list of nodes, edges, and its index
    make_paths_srv_ = nh.advertiseService("robot_paths", &MAMP::find_paths, this);
  }

  bool MAMP::find_paths(cs685::GetMultiPaths::Request &req, cs685::GetMultiPaths::Response &res) 
  {
    std::vector<std::pair<double, double>> starts;
    std::vector<std::pair<double, double>> goals;
    std::vector<int> goal_ids;
    std::vector<int> start_ids;
    std::pair<double, double> point1, point2;
    Node closest_node;
    
    std::ofstream myfile;
    myfile.open ("/home/dzungbui/ros_noetic/src/ccbs/src/paths.txt");
    for (int i =0; i< req.starts.size(); i++) {
      point1.first = (req.starts[i].pose.position.x +10.0)/0.05;
      point1.second = (req.starts[i].pose.position.y +9.0)/0.05;
      closest_node = map.get_closest_node(point1);
      point2.first = closest_node.i;
      point2.second = closest_node.j;      

      starts.push_back(point2);
      start_ids.push_back(closest_node.id);

      
      point1.first = (req.goals[i].pose.position.x +10.0)/0.05;
      point1.second = (req.goals[i].pose.position.y +9.)/0.05;
      closest_node = map.get_closest_node(point1);
      point2.first = closest_node.i;
      point2.second = closest_node.j;
   
      goals.push_back(point2);
      goal_ids.push_back(closest_node.id);
    }
    // set the coordinates of starts, goals, and their ids
    task.set_new_task(starts, goals, start_ids, goal_ids);
    solution = cbs.find_solution(map, task, config);  
    
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "map";
    for(int i = 0; i < int(solution.paths.size()); i++) {
      nav_msgs::Path path_n;
      myfile << "Trajectory: "+ std::to_string(i)+"\n";
      // adding the starting point
      p.pose.position.x = (req.starts[i].pose.position.x + 10.)/0.05;
      p.pose.position.y = (req.starts[i].pose.position.y +9.)/0.05;
      path_n.poses.push_back(p);

      myfile << p.pose.position.x << "  " << p.pose.position.y << "\n";

      auto iter = solution.paths[i].nodes.begin();
      while(iter != std::prev(solution.paths[i].nodes.end()))
      {
        p.pose.position.x = map.get_i(iter->id);
        p.pose.position.y = map.get_j(iter->id);
        p.header.stamp = ros::Time(iter->g);
        path_n.poses.push_back(p);
        myfile << p.pose.position.x << "  " << p.pose.position.y << "\n";
        iter++;
      }
      // adding the goal
      p.pose.position.x = (req.goals[i].pose.position.x +10.0)/0.05;
      p.pose.position.y = (req.goals[i].pose.position.y +9.0)/0.05;
      path_n.poses.push_back(p);
      
      myfile << p.pose.position.x << "  " << p.pose.position.y << "\n";
      res.paths.push_back(path_n);    
    }
    myfile.close();
    return true;
  }
}