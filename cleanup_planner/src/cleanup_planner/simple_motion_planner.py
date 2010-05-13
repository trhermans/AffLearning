# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Georgia Institute of Technology
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#  * Neither the name of the Georgia Institute of Technology nor the names of
#     its contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

import roslib; roslib.load_manifest('pioneer_control')
import rospy
from geometry_msgs.msg import Twist
from math import pi, atan2, hypot, fabs
from util import *

_TURN_ONLY_BEARING = pi / 6.0
_DRIVE_ONLY_BEARING  = pi / 36.0
_MAX_FORWARD_VEL = 0.3
_MIN_FORWARD_VEL = 0.05
_FORWARD_GAIN = 0.00025
_MAX_ROTATIONAL_VEL = 0.5
_MIN_ROTATIONAL_VEL = 0.1
_ROTATIONAL_GAIN = 0.25

class SimpleMotionPlanner:
    def __init__(self, eps_x, eps_y, eps_theta):
        self.moving = False
        self.sonar_avoid = False
        self.at_goal = False
        self.eps_x = eps_x
        self.eps_y = 0.01
        self.eps_theta = 0.01

    def get_velocity_command(self, current_pose, goal_pose,
                             use_goal_heading = False):
        """
        Given a current pose and a goal pose give a velocity command to begin to
        reach the goal pose.
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0

        bearing_to_goal = sub_pi_angle(atan2(current_pose.y - goal_pose.y,
                                             goal_pose.x - current_pose.x)
                                       - current_pose.theta)
        distance_to_goal = hypot(goal_pose.y - current_pose.y,
                                 goal_pose.x - current_pose.x)

        bearing_dir = sign(bearing_to_goal)
        bearing_mag = fabs(bearing_to_goal)

        self.at_goal = False

        # change this to use x_dist and y_dist
        if distance_to_goal < self.eps_x and distance_to_goal < self.eps_y:
            rospy.loginfo("Only have heading left to set")
            if use_goal_heading:
                h_diff = subPIangle(goal_pose.theta - current_pose.theta)
                if fabs(h_diff) > self.eps_theta:
                    cmd_vel.angular.z = sign(h_diff)*_MIN_ROTATIONAL_VEL
                else:
                    self.at_goal = True
            else:
                rospy.loginfo("Nothing left to set")
                self.at_goal = True

        elif bearing_mag > _TURN_ONLY_BEARING:
            cmd_vel.angular.z = bearing_dir*clip(_ROTATIONAL_GAIN*bearing_mag,
                                                 _MIN_ROTATIONAL_VEL,
                                                 _MAX_ROTATIONAL_VEL)
        else:
            cmd_vel.linear.x = clip(_FORWARD_GAIN * distance_to_goal,
                                    _MIN_FORWARD_VEL,
                                    _MAX_FORWARD_VEL)
            if bearing_mag > _DRIVE_ONLY_BEARING:
                cmd_vel.angular.z = bearing_dir*_MIN_ROTATIONAL_VEL
        return cmd_vel


    def stop_moving(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        return cmd_vel
