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
import rospy
import cleanup_constants as Constants
from geometry_msgs.msg import Twist
from math import hypot

#
# Navigation States
#

def stopped(controller):
    """
    State that does nothing, once the robot is stopped
    """
    return controller.stay()

def go_to_user_goal(controller):
    """
    State to drive the robot to a specified goal
    """
    controller.drive_to_location(controller.user_goal_pose)

    if controller.motion_planner.at_goal:
        return controller.goNow('stopped')

    return controller.stay()

def stop_robot(controller):
    """
    State to stop the robot from moving
    """
    controller.stop_driving()

    return controller.goNow('stopped')

def visit_objects(controller):
    """
    State to deal with visiting all of the objects in order given the planner
    """
    controller.determine_visit_path()

    return controller.goLater('visit_next_object')

def visit_next_object(controller):
    """
    Drive to the next object in the cleanup plan
    """
    if controller.firstFrame():

        if len(controller.visit_path) == 0:
            return controller.goLater('stopped')

        controller.current_object_loc = controller.visit_path.pop(0)

    controller.drive_to_location(controller.current_object_loc, True)

    if controller.motion_planner.at_goal:
        return controller.goLater('cleanup_object')

    return controller.stay()

#
# Cleanup Behaviors
#

def cleanup_object(controller):
    """
    State to organize the cleanup behavior selection once the robot has reached
    the object
    """
    if controller.firstFrame():
        controller.stop_driving()

    # TODO: Select based on affordances here
    # return controller.goLater('push_object')
    # return controller.goLater('roll_push_object')
    # return controller.goLater('drag_object')
    return controller.goLater('carry_object')

def push_object(controller):
    """
    Push the object into the cleanup zone.
    """
    if controller.firstFrame():
        controller.stop_driving()

    if not controller.in_cleanup_zone():
        # send back up command
        cmd_vel = Twist()
        cmd_vel.linear.x = Constants.PUSH_X_VEL
        controller.pioneer.vel_pub.publish(cmd_vel)
        return controller.stay()

    controller.stop_driving()

    return controller.goLater('back_off_object')

def roll_push_object(controller):
    """
    Shove the object so that it rolls.
    Do so on a trajectory towards the cleanup zone.
    """
    if controller.firstFrame():
        controller.odometry_start_pose = controller.pioneer.odometry_pose.pose
        controller.stop_driving()

    current_odo_pose = controller.pioneer.odometry_pose.pose
    odo_dist = hypot(current_odo_pose.pose.position.x -
                     controller.odometry_start_pose.pose.position.x,
                     current_odo_pose.pose.position.y -
                     controller.odometry_start_pose.pose.position.y)

    if Constants.PUSH_ROLL_DIST > odo_dist:
        # send back up command
        cmd_vel = Twist()
        cmd_vel.linear.x = Constants.PUSH_ROLL_X_VEL
        controller.pioneer.vel_pub.publish(cmd_vel)
        return controller.stay()

    controller.stop_driving()

    return controller.goLater('visit_next_object')

def carry_object(controller):
    """
    Pick up an object, drive it into the cleanup zone.
    """
    # Pickup object
    if not controller.pioneer.pickup_object():
        return controller.stay()

    # Drive to cleanup zone
    if not controller.in_cleanup_zone():
        return controller.goLater('drive_to_cleanup_zone')

    return controller.goLater('put_down_object')

def drag_object(controller):
    """
    Grab an object, drag / drive it into the cleanup zone.
    """
    # Pickup object
    if not controller.pioneer.grab_object():
        return controller.stay()

    # Drive to cleanup zone
    if not controller.in_cleanup_zone():
        return controller.goLater('drive_to_cleanup_zone')

    return controller.goLater('put_down_object')

def put_down_object(controller):
    """
    Put down an object then back up before going to the next object.
    """
    if not controller.pioneer.put_down_object():
        return controller.stay()

    # Back off object
    return controller.goLater('back_off_object')

#
# Helper states
#
def drive_to_cleanup_zone(controller):
    """
    State to drive to the cleanup zone, ignorant of the affordance being used
    Once it reaches its goal, transitions back to the previous state
    """
    if controller.firstFrame():
        controller.set_cleanup_goal_pose()
    controller.drive_to_location(controller.cleanup_goal_pose, False)

    if controller.motion_planner.at_goal:
        return controller.goNow(controller.lastDiffState)

    return controller.stay()

def back_off_object(controller):
    """
    Uses raw odometry measurements to back away from an object
    """
    if controller.firstFrame():
        controller.odometry_start_pose = controller.pioneer.odometry_pose.pose
        controller.stop_driving()

    current_odo_pose = controller.pioneer.odometry_pose.pose
    odo_dist = hypot(current_odo_pose.pose.position.x -
                     controller.odometry_start_pose.pose.position.x,
                     current_odo_pose.pose.position.y -
                     controller.odometry_start_pose.pose.position.y)

    if Constants.BACK_OFF_DIST > odo_dist:
        # send back up command
        cmd_vel = Twist()
        cmd_vel.linear.x = Constants.BACK_OFF_X_VEL
        controller.pioneer.vel_pub.publish(cmd_vel)
        return controller.stay()

    controller.stop_driving()
    return controller.goLater('visit_next_object')

#
# Robot setup states
#

def setup_robot(controller):
    """
    State to stop the robot
    """
    if controller.pioneer.gripper_open:
        return controller.goLater("finished_setup")

    if not controller.pioneer.gripper_moving:
        controller.pioneer.deploy_gripper()
    return controller.stay()

# State name alias
finished_setup = stopped
