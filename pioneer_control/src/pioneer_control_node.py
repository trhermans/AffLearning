#!/usr/bin/env python
import roslib; roslib.load_manifest('pioneer_control')
import rospy
from p2os.msg import GripperState, MotorState, PTZState
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
MOTORS_ON = 4
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
        if not self.moved_:
            rospy.loginfo("Moving gripper")
            rospy.loginfo(data.grip.state)
            rospy.loginfo(data.lift.state)
            self.gripControl(CMD)
            self.moved_ = True

    def motorCallback(self, motors):
        if motors.state == MOTORS_ON:
            # self.velControl()
            pass
        else:
            # Turn on the motors
            rospy.loginfo("Motors are off")
            motor_state = MotorState()
            motor_state.state = MOTORS_ON
            self.motor_pub_.publish(motor_state)

    def velControl(self):
        vel_cmd = Twist()
        vel_cmd.angular.z = -1.5
        vel_cmd.linear.x = 0.0
        self.vel_pub_.publish(vel_cmd)

    def ptzCallback(self, ptz_state):
        if ptz_state.zoom == 0:
            cmd = PTZState()
            cmd.pan = 90
            cmd.tilt = 30
            cmd.zoom = 3
            self.ptz_pub_.publish(cmd)

    def run(self):
        rospy.init_node("pioneer_control")

        self.motor_pub_ = rospy.Publisher("motor_state_cmd", MotorState)
        self.vel_pub_ = rospy.Publisher("velocity_cmd", Twist)
        self.gripper_pub_ = rospy.Publisher("gripper_cmd", GripperState)
        self.ptz_pub_ = rospy.Publisher("ptz_cmd", PTZState)

        rospy.Subscriber("gripper_status", GripperState, self.gripperCallback)
        rospy.Subscriber("motor_status", MotorState, self.motorCallback)
        rospy.Subscriber("ptz_status", PTZState, self.ptzCallback)
        rospy.spin()

if __name__ == '__main__':
    controller = PioneerControlNode()
    controller.moved_ = False
    try:
        controller.run()
    except rospy.ROSInterruptException: pass
