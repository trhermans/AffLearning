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

def stopped(controller):
    return controller.stay()

def setup_robot(controller):
    if controller.pioneer.gripper_open:
        return controller.goLater("finished_setup")

    if not controller.pioneer.gripper_moving:
        controller.pioneer.deploy_gripper()
    return controller.stay()


finished_setup = stopped

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
    State to deal with visiting all of the objects in order
    """
    controller.determine_visit_path()

    return controller.goLater('visit_next_object')

def visit_next_object(controller):
    if controller.firstFrame():

        if len(controller.visit_path) == 0:
            return controller.goLater('stopped')

        controller.current_object_loc = controller.visit_path.pop(0)

    controller.drive_to_location(controller.current_object_loc, True)

    if controller.motion_planner.at_goal:
        return controller.goLater('cleanup_object')

    return controller.stay()

def wait_at_object(controller):
    if controller.firstFrame():
        controller.stop_driving()

    if controller.counter > 100:
        return controller.goLater('visit_next_object')

    return controller.stay()

def cleanup_object(controller):
    if controller.firstFrame():
        controller.stop_driving()

    # TODO: Get affordances here
    if controller.counter > 100:
        return controller.goLater('carry_object')

    return controller.stay()

def drive_to_object(controller):
    """
    State for driving to an object
    """
    return controller.stay()

def push_object(controller):
    return controller.stay()

def roll_push_object(controller):
    return controller.stay()

def carry_object(controller):
    """
    """
    # Pickup object
    if not controller.pioneer.pickup_object():
        return controller.stay()

    # Drive to cleanup zone
    if not controller.in_cleanup_zone():
        return controller.goLater('drive_to_cleanup_zone')

    if not controller.pioneer.put_down_object():
        return controller.stay()

    return controller.goLater('visit_next_object')

def drag_object(controller):
    return controller.stay()

#
# Helper states
#
def drive_to_cleanup_zone(controller):
    """
    State to drive to the cleanup zone, ignorant of the affordance being used
    Once it reaches its goal, transitions back to the previous state
    """
    if controller.firstFrame():
        controller.cleanup_goal_pose = controller.planner.get_cleanup_pose()
    controller.drive_to_location(controller.cleanup_goal_pose)

    if controller.motion_planner.at_goal:
        return controller.goNow(controller.lastDiffState)

    return controller.stay()
