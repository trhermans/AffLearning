#!/usr/bin/env python
import roslib; roslib.load_manifest('pioneer_control')
import rospy
from p2os.msg import GripperState

def gripControl():
    pub = rospy.Publisher("grip_command", GripperState)
    rospy.init_node("grip_control")

    while not rospy.is_shutdown():
        grip_cmd = GripperState()
        grip_cmd.grip.state = 2 # Tell the gripper to open
        pub.publish(grip_cmd)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        gripControl()
    except rospy.ROSInterruptException: pass
