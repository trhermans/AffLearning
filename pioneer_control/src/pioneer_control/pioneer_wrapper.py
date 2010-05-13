#!/usr/bin/env python
import roslib; roslib.load_manifest('pioneer_control')
import rospy
import pioneer_command_constants as Commands
from p2os.msg import GripperState, MotorState, PTZState, BatteryState
from geometry_msgs.msg import Twist

class PioneerWrapper:
    def __init__(self):
        """
        Class to provide intermediate wrappers to perform standard tasks with
        the pionner.
        """
        # Setup publishers to the pioneer
        self.motor_pub = rospy.Publisher("cmd_motor_state", MotorState)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist)
        self.gripper_pub = rospy.Publisher("gripper_control", GripperState)
        self.ptz_pub = rospy.Publisher("ptz_control", PTZState)

        # Setup subscribers to the pioneer information
        rospy.Subscriber("motor_status", MotorState, self.motor_state_callback)
        rospy.Subscriber("battery_state", BatteryState,
                         self.battery_state_callback)
        rospy.Subscriber("gripper_status", GripperState,
                         self.gripper_state_callback)
        rospy.Subscriber("ptz_status", PTZState, self.ptz_state_callback)

        # Class state fields
        self.motor_state = MotorState()
        self.battery_state = BatteryState()
        self.ptz_state = PTZState()
        self.gripper_state = GripperState()

        self.gripper_moving = False
        self.gripper_open = False
        self.gripper_closed = False
        self.lift_moving = False
        self.gripper_moving = False
        self.motors_activated = False

    #
    # Callback functions
    #
    def motor_state_callback(self, msg):
        """
        Callback method to store the current state of the motors
        """
        self.motor_state.state = msg.state
        self.motors_activated = (self.motor_state.state == Commands.MOTORS_ON)

        if not self.motors_activated:
            self.activate_motors()

    def battery_state_callback(self, msg):
        """
        Callback method to store the current state of the battery
        """
        self.battery_state.voltage = self.msg.voltage

    def gripper_state_callback(self, msg):
        """
        Callback method to store the current state of the gripper
        """
        self.gripper_state.grip.state = msg.grip.state
        self.gripper_state.grip.dir = msg.grip.dir

        self.gripper_state.grip.inner_beam = msg.grip.inner_beam
        self.gripper_state.grip.outer_beam = msg.grip.outer_beam

        self.gripper_state.grip.left_contact = msg.grip.left_contact
        self.gripper_state.grip.right_contact = msg.grip.right_contact

        self.gripper_state.lift.state = msg.lift.state
        self.gripper_state.lift.dir = msg.lift.dir
        self.gripper_state.lift.position = msg.lift.position

        # Set query values
        self.gripper_moving = False
        self.gripper_closed = False
        self.gripper_open = False

        state = self.gripper_state.grip.state
        if state == GRIPPER_MOVING_STATE:
            self.gripper_moving = True
        elif state == GRIPPER_OPEN_STATE:
            self.gripper_closed = True
        elif state == GRIPPER_CLOSED_STATE:
            self.gripper.open = True

        self.lift_moving = False
        if self.gripper_state.lift.state == LIFT_MOVING_STATE:
            self.lift_moving = True

    def ptz_state_callback(self, msg):
        """
        Callback method to store the current state of the PTZ
        """
        self.ptz_state.pan = msg.pan
        self.ptz_state.tilt = msg.tilt
        self.ptz_state.zoom = msg.zoom

    #
    # Mid level gripper behaviors
    #
    def grab_object(self):
        if not gripper_moving:
            if gripper_open:
                self.close_gripper()
            else:
                return True
        return False

    def release_object(self):
        if not gripper_moving:
            if gripper_closed:
                self.open_gripper()
            else:
                return True
        return False

    def pickup_object(self):
        if self.grab_object():
            if not self.lift_moving:
                if self.gripper_state.lift.position < 1.0:
                    self.raise_gripper()
                else:
                    return True
        return False

    def put_down_object(self):
        if not self.lift_moving:
            if self.gripper_state.lift.position > 0.0:
                self.lower_gripper()
            else if self.release_object():
                return True
        return False

    #
    # Low level gripper behaviors
    #
    def deploy_gripper():
        cmd = GripperState()
        cmd.grip.state = Commands.GRIP_OPEN
        cmd.lift.state = Commands.LIFT_DOWN
        self.gripper_pub.publish(cmd)

    def store_gripper():
        cmd = GripperState()
        cmd.grip.state = Commands.GRIP_CLOSE
        cmd.lift.state = Commands.LIFT_UP
        self.gripper_pub.publish(cmd)

    def open_gripper():
        cmd = GripperState()
        cmd.grip.state = Commands.GRIP_OPEN
        self.gripper_pub.publish(cmd)

    def close_gripper():
        cmd = GripperState()
        cmd.grip.state = Commands.GRIP_CLOSE
        self.gripper_pub.publish(cmd)

    def raise_gripper():
        cmd = GripperState()
        cmd.lift.state = Commands.LIFT_UP
        self.gripper_pub.publish(cmd)

    def lower_gripper():
        cmd = GripperState()
        cmd.lift.state = Commands.LIFT_DOWN
        self.gripper_pub.publish(cmd)

    #
    # Motor state control
    #
    def activate_motors():
        m = MotorState()
        m.state = Commands.MOTORS_ON
        self.motor_pub.publish(m)

    def deactivate_motors():
        m = MotorState()
        m.state = Commands.MOTORS_OFF
        self.motor_pub.publish(m)
