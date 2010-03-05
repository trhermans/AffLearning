#!/usr/bin/env python
import roslib; roslib.load_manifest('pioneer_control')
import rospy
from p2os.msg import GripperState, MotorState
from geometry_msgs.msg import Twist

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

CMD = 1

class PioneerControlNode:
    def gripControl(self, i):
        grip_cmd = GripperState()
        if  i % 2 == 0:
            grip_cmd.grip.state = GRIP_OPEN
            grip_cmd.lift.state = LIFT_DOWN
        else:
            grip_cmd.grip.state = GRIP_CLOSE
            grip_cmd.lift.state = LIFT_UP

        self.gripper_pub_.publish(grip_cmd)

    def gripperCallback(self, data):
        rospy.loginfo(data.grip.state)
        rospy.loginfo(data.lift.state)
        if not self.moved_:
            rospy.loginfo("Moving gripper")
            self.gripControl(CMD)
            self.moved_ = True

        self.velControl()

    def velControl(self):
        # Turn on the motors
        motor_state = MotorState()
        motor_state.state = 4
        self.motor_pub_.publish(motor_state)

        vel_cmd = Twist()
        vel_cmd.angular.z = 1.5
        vel_cmd.linear.x = 0.1
        # self.vel_pub_.publish(vel_cmd)


    def run(self):
        rospy.init_node("pioneer_control")

        self.motor_pub_ = rospy.Publisher("motor_state_command", MotorState)
        self.vel_pub_ = rospy.Publisher("velocity_command", Twist)
        self.gripper_pub_ = rospy.Publisher("gripper_command", GripperState)

        rospy.Subscriber("gripper_status", GripperState, self.gripperCallback)

        rospy.spin()

if __name__ == '__main__':

    controller = PioneerControlNode()
    controller.moved_ = False
    try:
        controller.run()
    except rospy.ROSInterruptException: pass

