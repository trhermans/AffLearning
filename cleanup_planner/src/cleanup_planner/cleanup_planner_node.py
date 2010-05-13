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

import roslib; roslib.load_manifest('cleanup_planner')
import rospy
from pioneer_control import pioneer_wrapper
from cleanup_control import CleanupControl
from geometry_msgs.msg import Pose2D
from overhead_tracking.msg import CleanupObjectArray, CleanupZoneArray

class CleanupPlannerNode:
    """
    ROS node that spins a FSA to control for the cleanup task
    """
    def run(self):
        """
        Method to initialize node and then spin the cleanup planner FSA
        """
        rospy.init_node("cleanup_planner_node")

        self.pioneer = pioneer_wrapper.PioneerWrapper()

        rospy.Subscriber("robot_pose", Pose2D, self.robot_pose_callback)
        rospy.Subscriber("goal_pose", Pose2D, self.goal_pose_callback)
        rospy.Subscriber("cleanup_objects", CleanupObjectArray,
                         self.object_callback)
        rospy.Subscriber("cleanup_zones", CleanupZoneArray,
                         self.cleanup_zone_callback)

        self.control = CleanupControl(self)
        self.control.setPrintFunction(rospy.loginfo)

        rospy.on_shutdown(self.shutdown_hook)

        r = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            self.control.run()
            r.sleep()

    def robot_pose_callback(self, msg):
        self.control.robot_pose.x = msg.x
        self.control.robot_pose.y = msg.y
        self.control.robot_pose.theta = msg.theta

    def goal_pose_callback(self, msg):
        if (msg.x != self.control.user_goal_pose.x or
            msg.y != self.control.user_goal_pose.y):
            self.control.user_goal_pose.x = msg.x
            self.control.user_goal_pose.y = msg.y
            self.control.user_goal_pose.theta = msg.theta

            rospy.loginfo("Updated robot goal pose")

            #self.control.switchTo('go_to_user_goal')
            self.control.switchTo('visit_objects')

        if msg.x == -1337:
            self.control.switchTo('stop_robot')
            rospy.loginfo("Removed robot goal pose")

    def object_callback(self, msg):
        self.control.cleanup_objects = msg.objects

    def cleanup_zone_callback(self, msg):
        self.control.cleanup_zones = msg.zones

    def shutdown_hook(self):
        rospy.logdebug("Storing gripper on shutdown!")
        self.pioneer.store_gripper()
        rospy.sleep(2.0)

if __name__ == '__main__':
    controller = CleanupPlannerNode()

    try:
        controller.run()
    except rospy.ROSInterruptException: pass
