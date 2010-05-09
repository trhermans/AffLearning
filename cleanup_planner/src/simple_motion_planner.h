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

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

class SimpleMotionPlanner
{
 public:
  SimpleMotionPlanner();
  geometry_msgs::Twist getVelocityCommand(geometry_msgs::Pose2D robot_pose,
                                          geometry_msgs::Pose2D goal_pose,
                                          bool set_heading=false);
  geometry_msgs::Twist stopMoving();
  bool atGoalPose() { return at_goal_; }

 protected:
  geometry_msgs::Pose2D goal_pose_;
  geometry_msgs::Pose2D current_pose_;
  bool moving_;
  bool sonar_avoid_;
  float eps_x_;
  float eps_y_;
  float eps_theta_;
  bool at_goal_;
  static const double TURN_ONLY_BEARING;
  static const double DRIVE_ONLY_BEARING;
  static const double MAX_FORWARD_VEL;
  static const double MIN_FORWARD_VEL;
  static const double FORWARD_GAIN;
  static const double MAX_ROTATIONAL_VEL;
  static const double MIN_ROTATIONAL_VEL;
  static const double ROTATIONAL_GAIN;
};
