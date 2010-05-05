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
#include "simple_motion_planner.h"
#include <cmath>

#define TURN_ONLY_BEARING M_PI / 6.0 // 30.0 degrees
#define MAX_FORWARD_VEL 0.08
#define MIN_FORWARD_VEL 0.05
#define MAX_ROTATIONAL_VEL 0.5
#define MIN_ROTATIONAL_VEL 0.2

inline int sign(double x)
{
  if (x < 0)
    return -1;
  else if (x > 0)
    return 1;
  else
    return 0;
}

inline float clip(float val, float min_val, float max_val)
{
  if (val < min_val)
    return min_val;
  if (val > max_val)
    return max_val;
  return val;
}

double subPIangle(double theta)
{
  theta = std::fmod(theta, 2.0*M_PI);
  if( theta > M_PI) {
    theta -= 2.0*M_PI;
  }

  if( theta < -M_PI) {
    theta += 2.0*M_PI;
  }
  return theta;
}

double subPIdiff(double theta, double from_theta)
{
  while(abs(theta - from_theta) > M_PI)
  {
    if (theta > 0)
      theta -= 2.0*M_PI;
    else
      theta += 2.0*M_PI;
  }
  return (theta - from_theta);
}

SimpleMotionPlanner::SimpleMotionPlanner() :
    moving_(false), sonar_avoid_(false), eps_x_(0.3), eps_y_(0.3),
    eps_theta_(M_PI/8.0)
{
}
void SimpleMotionPlanner::setGoalPose(geometry_msgs::Pose2D goal_pose)
{
  goal_pose_.x = goal_pose.x;
  goal_pose_.y = goal_pose.y;
  goal_pose_.theta = goal_pose.theta;
}

geometry_msgs::Twist SimpleMotionPlanner::getVelocityCommand(
    geometry_msgs::Pose2D robot_pose)
{
  current_pose_ = robot_pose;
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;

  double bearing_to_goal = (atan2(robot_pose.y - goal_pose_.y,
                                  goal_pose_.x - robot_pose.x)
                            - robot_pose.theta);
  double distance_to_goal = hypot(goal_pose_.y - robot_pose.y,
                                  goal_pose_.x - robot_pose.x);

  int bearing_dir = sign(bearing_to_goal);
  double bearing_mag = abs(bearing_to_goal);

  // Check if we are at the goal
  if (abs(distance_to_goal) < eps_x_ &&
      abs(distance_to_goal) < eps_y_)
  {
    // if ( bearing_mag > eps_theta_)
    // {
    //   //cmd_vel.angular.z = bearing_to_goal;
    //   cmd_vel.angular.z = sign(bearing_to_goal)*MIN_ROTATIONAL_VEL;
    // }
  } // Check if we need to turn to the goal first
  else if( bearing_mag > TURN_ONLY_BEARING)
  {
    cmd_vel.angular.z = bearing_dir*MIN_ROTATIONAL_VEL;
  }
  else
  {
    cmd_vel.linear.x = clip(MAX_FORWARD_VEL / distance_to_goal,
                            MIN_FORWARD_VEL,
                            MAX_FORWARD_VEL);
    //cmd_vel.angular.z = bearing_dir*MIN_ROTATIONAL_VEL;
  }

  return cmd_vel;
}

geometry_msgs::Twist SimpleMotionPlanner::stopMoving()
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = 0;
  cmd_vel.angular.z = 0;
  return cmd_vel;
}
