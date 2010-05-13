#!/usr/bin/env python
import roslib; roslib.load_manifest('pioneer_control')
import rospy
from pioneer_control import pioneer_wrapper
from cleanup_planner import CleanupControl
#from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from cleanup_planner.msg import CleanupObjectArray, CleanupZoneArray

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
        rospy.Subscriber("cleanup_zones", Pose2D, self.cleanup_zone_callback)

        # Move this stuff into the cleanup_planner FSA
        self.cleanup_objects = CleanupObjectArray()
        self.cleanup_zones = CleanupZoneArray()
        self.robot_pose = Pose2D()
        self.user_goal_pose = Pose2D()

        self.control = CleanupControl(self)
        self.control.setPrintFunction(rospy.loginfo)

        while not rospy.is_shutdown():
            self.planner.run()
            rospy.sleep(0.001)

    def robot_pose_callback(self, msg):
        self.robot_pose.x = msg.x
        self.robot_pose.y = msg.y
        self.robot_pose.theta = msg.theta

    def goal_pose_callback(self, msg):
        if msg.x != self.user_goal_pose.x or msg.y != self.user_goal_pose.y:
            self.user_goal_pose.x = msg.x
            self.user_goal_pose.y = msg.y
            self.user_goal_pose.theta = msg.theta
            rospy.loginfo("Updated robot goal pose")
            self.planner.switchTo('go_to_user_goal')

        if msg.x == -1337:
            self.planner.switchTo('stop_robot')
            rospy.loginfo("Removed robot goal pose")

    def object_callback(self, msg):
        self.cleanup_objects = msg.objects

    def cleanup_zone_callback(self, msg):
        pass

if __name__ == '__main__':
    controller = CleanupPlannerNode()

    try:
        controller.run()
    except rospy.ROSInterruptException: pass
