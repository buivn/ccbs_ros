/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Hoang-Dung Bui
*********************************************************************/
#include <cs685/local_controller.hpp>
#include <move_base_msgs/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace cs685 {

  Local_Controller::Local_Controller(tf2_ros::Buffer& tf):
    tf_(tf), as_(NULL),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    controller_plan_(NULL) 
    // setup_(false), p_freq_change_(false), c_freq_change_(false),
    // controller_frequency_(0.1)
  {
    as_ = new move_controlActionServer(ros::NodeHandle(), "/local_control", 
                                          [this](auto& path){ executeCb(path); }, false);

    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;

    //get some parameters that will be global to the move base node
    std::string local_planner;
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
    private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));
    private_nh.param("controller_frequency", controller_frequency_, 10.0);
    //set up plan triple buffer
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

     //for commanding the base
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //create the ros wrapper for the planner's costmap.
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //create a local planner
    try {
      lp_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      lp_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
      ROS_FATAL("Failed to create the %s planner, is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }
    // Start actively updating costmaps based on sensor data
    controller_costmap_ros_->start();
    planner_costmap_ros_->start();

    //we're all set up now so we can start the action server
    as_->start();
  }


  Local_Controller::~Local_Controller(){
    if(as_ != NULL)
      delete as_;
    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;
    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;
    
    delete controller_plan_;
    lp_.reset();
  }


  void Local_Controller::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }

  
  bool Local_Controller::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
      ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
      return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6){
      ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
      return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3){
      ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
      return false;
    }

    return true;
  }

  void Local_Controller::executeCb(const cs685::move_controlGoalConstPtr& path)
  {
    std::cout << "The frequency is: " << controller_frequency_ << std::endl;
    ros::Rate r(controller_frequency_);
    if(shutdown_costmaps_){
      ROS_DEBUG_NAMED("cs685","Starting up costmaps that were shut down previously");
      controller_costmap_ros_->start();
      planner_costmap_ros_->start();
    }

    ros::NodeHandle n;
    while(n.ok())
    {
      //for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      nav_msgs::Path p = path->path;
      bool done = executeCycle(p);

      // if we're done, then we'll return from execute
      if(done)
        return;

      //check if execution of the goal has completed in some way

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("cs685","Full control cycle time: %.9f\n", t_diff.toSec());

      r.sleep();
    }

    //if the node is killed then we'll abort and return
    as_->setAborted(cs685::move_controlResult(), "Aborting on the goal because the node has been killed");
    return;
  }

  double Local_Controller::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
  {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
  }

  bool Local_Controller::executeCycle(nav_msgs::Path & path){
    // this lock be is sure that the configuration is not able to change during this function
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;
    
    // set the set of poses
    std::vector<geometry_msgs::PoseStamped> plan;
    for (auto p: path.poses)
    {
      p.pose.position.x = -10.0 + p.pose.position.x*0.05 - 0.0;
      p.pose.position.y = -10.0 + p.pose.position.y*0.05 +1.;  
      plan.push_back(p);
    }


    //update feedback to correspond to our curent position
    geometry_msgs::PoseStamped global_pose;
    getRobotPose(global_pose, controller_costmap_ros_);

    //push the feedback out
    cs685::move_controlFeedback feedback;
    feedback.current_pose = global_pose;
    as_->publishFeedback(feedback);



    if(!lp_->setPlan(plan)){
      //ABORT and SHUTDOWN COSTMAPS
      ROS_ERROR("Failed to pass global plan to the controller, aborting.");
      publishZeroVelocity();
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
      as_->setAborted(cs685::move_controlResult(), "Failed to pass global plan to the controller.");
      return true;
    }

    //check to see if we've reached our goal
    // if(lp_->isGoalReached()){
    if (((abs(global_pose.pose.position.x-plan.back().pose.position.x) < 0.2)) and 
      (abs(global_pose.pose.position.y-plan.back().pose.position.y) < 0.2))
    {
      ROS_DEBUG_NAMED("cs685","Goal reached!");
      publishZeroVelocity();
      planner_costmap_ros_->stop();
      controller_costmap_ros_->stop();
      as_->setSucceeded(cs685::move_controlResult(), "Goal reached.");
      return true;
    }

    if(lp_->computeVelocityCommands(cmd_vel)){
      ROS_DEBUG_NAMED( "cs685", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                       cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
      //make sure that we send the velocity command to the base
      vel_pub_.publish(cmd_vel);
    }
    else {
      ROS_DEBUG_NAMED("cs685", "The local planner could not find a valid plan.");

      publishZeroVelocity();
    }
    //we aren't done yet
    return false;
  }



  bool Local_Controller::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
  {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
      tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
      return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
      ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
      return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (!global_pose.header.stamp.isZero() &&
        current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
      ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                        "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                        current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
      return false;
    }

    return true;
  }
};
