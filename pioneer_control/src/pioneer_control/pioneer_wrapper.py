#!/usr/bin/env python
import roslib; roslib.load_manifest('pioneer_control')
import rospy
import pioneer_command_constants as Commands
from p2os_driver.msg import GripperState, MotorState, PTZState, BatteryState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PioneerWrapper:
    def __init__(self):
        """
        Class to provide intermediate wrappers to perform standard tasks with
        the pionner.
        """
        # Setup publishers to the pioneer
        self.motor_pub = rospy.Publisher("motor_state_cmd", MotorState)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist)
        self.gripper_pub = rospy.Publisher("gripper_cmd", GripperState)
        self.ptz_pub = rospy.Publisher("ptz_cmd", PTZState)

        # Setup subscribers to the pioneer information
        rospy.Subscriber("motor_state", MotorState, self.motor_state_callback)
        rospy.Subscriber("battery_state", BatteryState,
                         self.battery_state_callback)
        rospy.Subscriber("gripper_state", GripperState,
                         self.gripper_state_callback)
        rospy.Subscriber("ptz_status", PTZState, self.ptz_state_callback)
        rospy.Subscriber("pose", Odometry, self.odometry_callback)

        # Class state fields
        self.motor_state = MotorState()
        self.battery_state = BatteryState()
        self.ptz_state = PTZState()
        self.gripper_state = GripperState()
        self.odometry_pose = Odometry()

        # Query variables
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
        self.battery_state.voltage = msg.voltage

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
        if state == Commands.GRIPPER_OPEN_STATE:
            self.gripper_open = True
        elif state == Commands.GRIPPER_CLOSED_STATE:
            self.gripper_closed = True
        elif state == Commands.GRIPPER_MOVING_STATE:
            self.gripper_moving = True

        self.lift_moving = False
        if self.gripper_state.lift.state == Commands.LIFT_MOVING_STATE:
            self.lift_moving = True

    def ptz_state_callback(self, msg):
        """
        Callback method to store the current state of the PTZ
        """
        self.ptz_state.pan = msg.pan
        self.ptz_state.tilt = msg.tilt
        self.ptz_state.zoom = msg.zoom

    def odometry_callback(self, msg):
        """
        Callback method to store the current odometry state
        """
        self.odometry_pose.pose = msg.pose
        self.odometry_pose.twist = msg.twist

    #
    # Mid level gripper behaviors
    #
    def grab_object(self):
        if not self.gripper_moving:
            if self.gripper_open:
                self.close_gripper()
            else:
                return True
        return False

    def release_object(self):
        if not self.gripper_moving:
            if self.gripper_closed:
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
            elif self.release_object():
                return True
        return False

    #
    # Low level gripper behaviors
    #
    def deploy_gripper(self):
        cmd = GripperState()
        cmd.grip.state = Commands.GRIP_OPEN
        cmd.lift.state = Commands.LIFT_DOWN
        self.gripper_pub.publish(cmd)

    def store_gripper(self):
        cmd = GripperState()
        cmd.grip.state = Commands.GRIP_CLOSE
        cmd.lift.state = Commands.LIFT_UP
        self.gripper_pub.publish(cmd)

    def open_gripper(self):
        cmd = GripperState()
        cmd.grip.state = Commands.GRIP_OPEN
        self.gripper_pub.publish(cmd)

    def close_gripper(self):
        cmd = GripperState()
        cmd.grip.state = Commands.GRIP_CLOSE
        self.gripper_pub.publish(cmd)

    def raise_gripper(self):
        cmd = GripperState()
        cmd.lift.state = Commands.LIFT_UP
        self.gripper_pub.publish(cmd)

    def lower_gripper(self):
        cmd = GripperState()
        cmd.lift.state = Commands.LIFT_DOWN
        self.gripper_pub.publish(cmd)

    #
    # Motor state control
    #
    def activate_motors(self):
        m = MotorState()
        m.state = Commands.MOTORS_ON
        self.motor_pub.publish(m)

    def deactivate_motors(self):
        m = MotorState()
        m.state = Commands.MOTORS_OFF
        self.motor_pub.publish(m)
