#!/usr/bin/env python
import roslib; roslib.load_manifest('pioneer_control')
import rospy
from p2os.msg import GripperState

# Gripper commands
GRIP_OPEN   = 1
GRIP_CLOSE  = 2
GRIP_STOP   = 3
LIFT_UP     = 4
LIFT_DOWN   = 5
LIFT_STOP   = 6
GRIP_STORE  = 7
GRIP_DEPLOY = 8
GRIP_HALT   = 15
GRIP_PRESS  = 16
LIFT_CARRY  = 17

def gripControl():
    pub = rospy.Publisher("grip_command", GripperState)
    rospy.init_node("grip_control")

    # Run a single command to close and lift the gripper
    grip_cmd = GripperState()
    grip_cmd.grip.state = GRIP_CLOSE
    grip_cmd.lift.state = LIFT_UP
    pub.publish(grip_cmd)

if __name__ == '__main__':
    try:
        gripControl()
    except rospy.ROSInterruptException: pass





