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
#include "pioneer_wrapper.h"
#include "overhead_tracking/CleanupObjectArray.h"
#include "overhead_tracking/CleanupZoneArray.h"
#include "overhead_tracking/CleanupObject.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <map>
#include <cfloat>

class CleanupPlannerNode
{
 public:
  //
  // Constructor
  //
  CleanupPlannerNode(ros::NodeHandle &n) :
      n_(n), pioneer_(n_), updated_goal_(false), have_goal_(false)
  {
    cleanup_objs_sub_ = n_.subscribe("cleanup_objects", 1,
                                     &CleanupPlannerNode::objectCallback, this);
    robot_pose_sub_ = n_.subscribe("robot_pose", 1,
                                   &CleanupPlannerNode::robotPoseCallback,
                                   this);
    goal_pose_sub_ = n_.subscribe("goal_pose", 1,
                                  &CleanupPlannerNode::goalPoseCallback, this);
    cleanup_zone_sub_ = n_.subscribe("cleanup_zones", 1,
                                     &CleanupPlannerNode::cleanupZoneCallback,
                                     this);
    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  }

  //
  // ROS Callback Methods
  //
  void objectCallback(const overhead_tracking::CleanupObjectArrayConstPtr &msg)
  {
    cleanup_objects_ = msg->objects;
  }

  void robotPoseCallback(const geometry_msgs::Pose2DConstPtr &msg)
  {
    robot_pose_.x = msg->x;
    robot_pose_.y = msg->y;
    robot_pose_.theta = msg->theta;
  }

  void goalPoseCallback(const geometry_msgs::Pose2DConstPtr &msg)
  {
    if(msg->x != user_goal_pose_.x || msg->y != user_goal_pose_.y)
    {
      user_goal_pose_.x = msg->x;
      user_goal_pose_.y = msg->y;
      user_goal_pose_.theta = msg->theta;
      updated_goal_ = true;
      have_goal_ = true;
      ROS_INFO("Updated robot goal pose");
    }
    if (user_goal_pose_.x == -1337)
    {
      have_goal_ = false;
      updated_goal_ = true;
      ROS_INFO("Removed robot goal pose");
    }
  }

  void cleanupZoneCallback(
      const overhead_tracking::CleanupZoneArrayConstPtr &msg)
  {
  }

  //
  // Main Control Loop
  //
  void spin()
  {
    //
    // Initialize Robot
    //
#ifndef TEST_CLEANUP_TSP
    if(n_.ok())
    {
      while(! pioneer_.gripperOpen() && ! pioneer_.gripperMoving())
      {
        pioneer_.deployGripper();
        ros::spinOnce();
      }
    }
#endif

    //
    // Main Control loop
    //
    while(n_.ok())
    {
      // Drive to the user defined goal pose for now
      // TODO: Replace this with the TSP formulation of visiting objects
      // May need to add a state machine in order to accomadate this easily...
      if(have_goal_)
      {
#ifdef TEST_CLEANUP_TSP
        std::vector<int> visit_path = orderObjectVisits();
        for (unsigned int i = 0; i < visit_path.size(); ++i)
        {
          ROS_INFO("[%d]: %d", i, visit_path[i]);
        }
        have_goal_ = false;
#else
        if(driveToLocation(user_goal_pose_, false))
        {
          have_goal_ = false;
        }
#endif
      }
      else if (updated_goal_)
      {
        geometry_msgs::Twist cmd_vel =  mp_.stopMoving();
        cmd_vel_pub_.publish(cmd_vel);
        updated_goal_ = false;
        ROS_INFO("Stopped Robot!");
      }

      ros::spinOnce();
    }
  }

  //
  // Mid level control functions
  //
  bool driveToLocation(geometry_msgs::Pose2D goal, bool have_heading)
  {
    geometry_msgs::Twist cmd_vel = mp_.getVelocityCommand(robot_pose_,
                                                          goal,
                                                          have_heading);
    cmd_vel_pub_.publish(cmd_vel);
    if (mp_.atGoalPose())
      return true;

    return false;
  }

  std::vector<int> orderObjectVisits()
  {
    std::vector<int> cleanup_path;
    std::map<int, unsigned int> cleanup_ids;

    // TODO: prune objects which lie inside the cleanup zone
    for(unsigned int i = 0; i < cleanup_objects_.size(); ++i)
    {
      cleanup_ids[cleanup_objects_[i].object_id] = i;
    }

#ifdef TEST_CLEANUP_TSP
    geometry_msgs::Pose2D active_pose = user_goal_pose_;
#else
    geometry_msgs::Pose2D active_pose = robot_pose_;
#endif

    for(unsigned int i = 0; i < cleanup_objects_.size(); ++i)
    {
      double min_dist = DBL_MAX;
      int min_id = -1;
      std::map<int, unsigned int>::iterator it;
      for (it = cleanup_ids.begin(); it != cleanup_ids.end(); ++it)
      {
        // TODO: Add penalty for heading; calculate time to position, not dist
        double dist = hypot(active_pose.y -
                            cleanup_objects_[it->first].pose.y,
                            active_pose.x -
                            cleanup_objects_[it->first].pose.x);
        if (dist < min_dist)
        {
          min_dist = dist;
          min_id = it->first;
        }
      }

      active_pose = cleanup_objects_[cleanup_ids[min_id]].pose;
      cleanup_path.push_back(min_id);
      cleanup_ids.erase(min_id);
    }
    return cleanup_path;
  }

  //
  // Behavior functions
  //

  bool driveToObject(overhead_tracking::CleanupObject& obj)
  {
    // TODO: Drive to a location near the object
    bool at_obj = driveToLocation(obj.pose, false);

    if (at_obj)
    {
      // TODO: Align to be pointing at the object
    }
    return at_obj;
  }

  void pushObject(overhead_tracking::CleanupObject& obj)
  {
    // Assume in line with object
    // TODO: Drive until object is inside of a cleanup zone
  }

  void rollPushObject(overhead_tracking::CleanupObject& obj)
  {
    // Assume in line with object
    // TODO: Drive for a specific amount of time / distance, then stop
  }

  bool carryObject(overhead_tracking::CleanupObject& obj)
  {
    if (!pioneer_.pickupObject())
      return false;
    // TODO: drive to the goal zone
    return true;
  }

  bool dragObject(overhead_tracking::CleanupObject& obj)
  {
    if (!pioneer_.grabObject())
      return false;

    // TODO: drive to the goal zone
    return true;
  }

 protected:
  // Class members
  ros::Subscriber cleanup_objs_sub_;
  ros::Subscriber robot_pose_sub_;
  ros::Subscriber goal_pose_sub_;
  ros::Subscriber cleanup_zone_sub_;

  ros::Publisher cmd_vel_pub_;

  geometry_msgs::Pose2D user_goal_pose_;
  geometry_msgs::Pose2D robot_pose_;

  ros::NodeHandle n_;

  SimpleMotionPlanner mp_;
  PioneerWrapper pioneer_;
  std::vector<overhead_tracking::CleanupObject> cleanup_objects_;

  bool updated_goal_;
  bool have_goal_;
};

/**
 * Main control point for the cleanup planner ros node
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cleanup_planner");
  ros::NodeHandle n;
  CleanupPlannerNode cleanup_node(n);
  cleanup_node.spin();
  return 0;
}
