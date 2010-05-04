/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Georgia Institute of Technology
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
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
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
 *********************************************************************/

#include "ros/ros.h"
#include "simple_motion_planner.h"
#include "overhead_tracking/CleanupObjectArray.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

class CleanupPlannerNode
{
 public:
  CleanupPlannerNode(ros::NodeHandle &n) :
      n_(n), updated_goal_(false), have_goal_(false)
  {
    cleanup_objs_sub_ = n_.subscribe("cleanup_objs", 1,
                                     &CleanupPlannerNode::objectCallback, this);
    robot_pose_sub_ = n_.subscribe("robot_pose", 1,
                                   &CleanupPlannerNode::robotPoseCallback,
                                   this);
    goal_pose_sub_ = n_.subscribe("goal_pose", 1,
                                  &CleanupPlannerNode::goalPoseCallback, this);
    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  }

  void objectCallback(const overhead_tracking::CleanupObjectArrayConstPtr &msg)
  {
  }

  void robotPoseCallback(const geometry_msgs::Pose2DConstPtr &msg)
  {
    robot_pose_.x = msg->x;
    robot_pose_.y = msg->y;
    robot_pose_.theta = msg->theta;
  }

  void goalPoseCallback(const geometry_msgs::Pose2DConstPtr &msg)
  {
    if(msg->x != goal_pose_.x || msg->y != goal_pose_.y)
    {
      goal_pose_.x = msg->x;
      goal_pose_.y = msg->y;
      goal_pose_.theta = msg->theta;
      updated_goal_ = true;
      have_goal_ = true;
      mp_.setGoalPose(goal_pose_);
      ROS_INFO("Updated robot goal pose");
    }
    if (goal_pose_.x == -1337)
      have_goal_ = false;
  }

  void spin()
  {
    while(n_.ok())
    {
      if(have_goal_)
      {
        geometry_msgs::Twist cmd_vel = mp_.getVelocityCommand(robot_pose_);
        cmd_vel_pub_.publish(cmd_vel);
      }

      ros::spinOnce();
    }
  }

 protected:
  ros::Subscriber cleanup_objs_sub_;
  ros::Subscriber robot_pose_sub_;
  ros::Subscriber goal_pose_sub_;
  ros::Publisher cmd_vel_pub_;
  geometry_msgs::Pose2D goal_pose_;
  geometry_msgs::Pose2D robot_pose_;
  ros::NodeHandle n_;

  SimpleMotionPlanner mp_;

  bool updated_goal_;
  bool have_goal_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cleanup_planner");
  ros::NodeHandle n;
  CleanupPlannerNode cleanup_node(n);
  cleanup_node.spin();
}
