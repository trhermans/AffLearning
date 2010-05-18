#!/usr/bin/env python
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

import FSA
import cleanup_states
import simple_motion_planner
import greedy_cleanup_planner
import roslib; roslib.load_manifest('cleanup_planner')
import rospy
from geometry_msgs.msg import Pose2D
from overhead_tracking.msg import CleanupObjectArray, CleanupZoneArray
from math import pi
from geometry_testing import point_in_zone

class CleanupControl(FSA.FSA):
    def __init__(self, cleanup_node):
        FSA.FSA.__init__(self, cleanup_node)
        self.cleanup_node = cleanup_node
        self.addStates(cleanup_states)
        self.currentState = 'setup_robot'
        self.setName('CleanupControl')
        self.setPrintStateChanges(True)
        self.stateChangeColor = 'purple'
        self.pioneer = self.cleanup_node.pioneer

        # Planners
        self.motion_planner = simple_motion_planner.SimpleMotionPlanner()
        self.cleanup_planner = greedy_cleanup_planner.GreedyCleanupPlanner()

        # Planner provided information
        self.visit_path = []
        self.current_object_loc = None

        # ROS provided state variables
        self.cleanup_objects = CleanupObjectArray()
        self.robot_pose = Pose2D()
        self.cleanup_goal_pose = Pose2D()
        self.user_goal_pose = Pose2D()

    def drive_to_location(self, goal_pose, use_heading = False):
        """
        Takes a goal pose, uses the class's motion planner to calculate a
        command velocity and publishes the velocity to the robot
        """
        cmd_vel = self.motion_planner.get_velocity_command(self.robot_pose,
                                                           goal_pose,
                                                           use_heading)
        self.pioneer.vel_pub.publish(cmd_vel)

    def stop_driving(self):
        """
        Tell the robot to stop driving
        Cleanup anything else we need to do when stopping
        """
        cmd_vel = self.motion_planner.stop_moving()
        self.pioneer.vel_pub.publish(cmd_vel)

    def determine_visit_path(self):
        """
        Determine the order in which to cleanup the objects and the locations to
        best manipulate them
        """
        self.visit_path = self.cleanup_planner.get_object_ordering(
            self.cleanup_objects, self.robot_pose)

    def set_cleanup_zones(self, cleanup_zones):
        """
        Set the cleanup zones for the cleanup planner
        """
        self.cleanup_planner.cleanup_zones = cleanup_zones

    def in_cleanup_zone(self):
        """
        Determine if the robot is currently in the cleanup zone
        """
        return point_in_zone(self.robot_pose,
                             self.cleanup_planner.cleanup_zones)

    def set_cleanup_goal_pose(self):
        self.cleanup_goal_pose = self.cleanup_planner.get_cleanup_pose(
            self.robot_pose)

