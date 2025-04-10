#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cs685/move_controlAction.h>
#include <cs685/move_controlGoal.h>
#include <cs685/GetMultiPaths.h>


// class move_controlClient()


int main (int argc, char **argv)
{
  ros::init(argc, argv, "robot_move_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<cs685::move_controlAction> ac2("/robot2/local_control", true);
  actionlib::SimpleActionClient<cs685::move_controlAction> ac3("/robot3/local_control", true);

  ros::NodeHandle n;
  ros::ServiceClient MC_client = n.serviceClient<cs685::GetMultiPaths>("robot_paths");
  geometry_msgs::PoseStamped sg;
  cs685::GetMultiPaths srv;
  // For the starting poses
  sg.pose.position.x = -2;
  sg.pose.position.y = -0.5;
  sg.pose.position.z = 0;
  sg.pose.orientation.x = 0;
  sg.pose.orientation.y = 0;
  sg.pose.orientation.z = 0;
  sg.pose.orientation.w = 1;
  srv.request.starts.push_back(sg);
  // robot 2
  sg.pose.position.x = 1.0;
  sg.pose.position.y = 0.5;
  srv.request.starts.push_back(sg);

  // For the goals
  // robot 1
  sg.pose.position.x = -0.5;
  sg.pose.position.y = 0.0;
  srv.request.goals.push_back(sg);
  // robot 2
  sg.pose.position.x = -1.5;
  sg.pose.position.y = -0.5;
  srv.request.goals.push_back(sg);


// action_goal_pub_ = action_nh.advertise<move_base_msgs::MoveBaseActionGoal>("goal", 1);
  // std::cout << "check step by step ------- " << std::endl;
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac3.waitForServer();
  ac2.waitForServer(); //will wait for infinite time

  // send a goal to the action
  cs685::move_controlGoal path2, path3;
  if (MC_client.call(srv))
  {
    // ROS_INFO("Sum: %ld", (long int)srv.response.sum)
    // for (int i = 0; i< srv.response.size(); i++) {

    // }
    ROS_INFO("Action server started, sending goal.");
    path2.path.header.frame_id = srv.response.paths[0].header.frame_id;
    for (int i =0; i< srv.response.paths[0].poses.size();i++) {
      path2.path.poses.push_back(srv.response.paths[0].poses[i]);
    }

    path3.path.header.frame_id = srv.response.paths[1].header.frame_id;
    for (int i =0; i< srv.response.paths[1].poses.size();i++) {
      path3.path.poses.push_back(srv.response.paths[1].poses[i]);
    }
    ac2.sendGoal(path2);
    ac3.sendGoal(path3);
    ROS_INFO("Sending goal - successfully.");
  }
  else
  {
    ROS_ERROR("Failed to call service get-paths");
    return 1;
  }  
  //wait for the action to return
  bool finished_before_timeout2 = ac2.waitForResult(ros::Duration(50.0));

  bool finished_before_timeout3 = ac3.waitForResult(ros::Duration(50.0));




  if (finished_before_timeout2)
  {
    actionlib::SimpleClientGoalState state2 = ac2.getState();
    ROS_INFO("Action 2 finished: %s",state2.toString().c_str());
  }
  else
    ROS_INFO("Action 2 did not finish before the time out.");

  if (finished_before_timeout3)
  {
    actionlib::SimpleClientGoalState state3 = ac3.getState();
    ROS_INFO("Action 3 finished: %s",state3.toString().c_str());
  }
  else
    ROS_INFO("Action 3 did not finish before the time out.");

  //exit
  return 0;
}
