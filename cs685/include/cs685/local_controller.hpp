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

*********************************************************************/
#ifndef LOCAL_CONTROLLER_H_
#define LOCAL_CONTROLLER_H_

// #include <cs685/cs685.h>
#include <cmath>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <nav_core/base_local_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <pluginlib/class_loader.hpp>

#include <actionlib/server/simple_action_server.h>
#include <cs685/move_controlGoal.h>
#include <cs685/move_controlResult.h>
#include <cs685/move_controlFeedback.h>
#include <cs685/move_controlAction.h>

#include <dynamic_reconfigure/server.h>
#include "cs685/CS685Config.h"

namespace cs685 {
  //typedefs to help us out with the action server so that we don't hace to type so much
  typedef actionlib::SimpleActionServer<cs685::move_controlAction> move_controlActionServer;

  class Local_Controller {
    public:  
      Local_Controller(std::string rID, tf2_ros::Buffer& tf);
      Local_Controller(tf2_ros::Buffer& tf);
      ~Local_Controller();
      void publishZeroVelocity();
      bool isQuaternionValid(const geometry_msgs::Quaternion& q);
      void executeCb(const cs685::move_controlGoalConstPtr& path);
      double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2);
      bool executeCycle(nav_msgs::Path & path);
      bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);
    
    private:
      move_controlActionServer *as_;
      std::string robotID;
      tf2_ros::Buffer& tf_;

      std::vector<geometry_msgs::PoseStamped> *controller_plan_;
      // Robot_State robot_state_;
      boost::recursive_mutex configuration_mutex_;

      boost::shared_ptr<nav_core::BaseLocalPlanner> lp_;
      costmap_2d::Costmap2DROS *controller_costmap_ros_, *planner_costmap_ros_;
      ros::Publisher vel_pub_;
      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
      std::string robot_base_frame_, global_frame_;
      double controller_frequency_;
      bool shutdown_costmaps_;

      
      // checking
      

      // std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
      // std::vector<std::string> recovery_behavior_names_;
      // unsigned int recovery_index_;

      // geometry_msgs::PoseStamped global_pose_;
      // double planner_frequency_, controller_frequency_, inscribed_radius_, circumscribed_radius_;
      // double planner_patience_, controller_patience_;
      // int32_t max_planning_retries_;
      // uint32_t planning_retries_;
      // double conservative_reset_dist_, clearing_radius_;
      // ros::Publisher current_goal_pub_, vel_pub_, action_goal_pub_, recovery_status_pub_;
      // ros::Subscriber goal_sub_, goals_sub_;
      // ros::ServiceServer make_plan_srv_, clear_costmaps_srv_;
      // bool shutdown_costmaps_, clearing_rotation_allowed_, recovery_behavior_enabled_;
      // bool make_plan_clear_costmap_, make_plan_add_unreachable_goal_;
      // double oscillation_timeout_, oscillation_distance_;


      // RecoveryTrigger recovery_trigger_;

      // ros::Time last_valid_plan_, last_valid_control_, last_oscillation_reset_;
      // geometry_msgs::PoseStamped oscillation_pose_;
      // pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
      // pluginlib::ClassLoader<cs685_navfn::CS685_NavfnROS> bgp_loaders_;
      // pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_, blp_loader1_, blp_loader2_;
      // pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;

      //set up plan triple buffer
      // std::vector<geometry_msgs::PoseArray> *planner_plans_;
      
      // geometry_msgs::PoseArray *latest_plan1_, *latest_plan2_;
      // geometry_msgs::PoseArray *controller_plan1_, *controller_plan2_;
      // std::vector<geometry_msgs::PoseStamped>* planner_goals_;

      
      // std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      // std::vector<geometry_msgs::PoseStamped> *latest_plan_;


      //set up the planner's thread
      // bool runPlanner_, runPlanners_;
      // boost::recursive_mutex planner_mutex_;
      // boost::condition_variable_any planner_cond_, planners_cond_;
      // geometry_msgs::PoseStamped planner_goal_;
      
      // geometry_msgs::PoseArray planner_goals_;;
      // boost::thread* planner_threads_, * planner_thread_;



      // dynamic_reconfigure::Server<cs685::CS685Config> *dsrv_;
      
      // void reconfigureCB(cs685::CS685Config &config, uint32_t level);

      // cs685::CS685Config last_config_;
      // cs685::CS685Config default_config_;
      // bool setup_, p_freq_change_, c_freq_change_;
      // bool new_global_plan_, goals_arrived_;
      
};

};

#endif