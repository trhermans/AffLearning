/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Georgia Institute of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef pioneer_wrapper_h_DEFINED
#define pioneer_wrapper_h_DEFINED

#include "ros/ros.h"
#include "p2os/GripperState.h"
#include "p2os/MotorState.h"
#include "p2os/PTZState.h"

class PioneerWrapper
{
 public:
  //
  // Constructors
  //
  PioneerWrapper(ros::NodeHandle &n);

  //
  // Callback Functions
  //
  void gripperStateCallback(const p2os::GripperStateConstPtr &msg);
  void motorStateCallback(const p2os::MotorStateConstPtr &msg);

  //
  // Simple Command Wrappers
  //
  void deployGripper();
  void storeGripper();
  void openGripper();
  void closeGripper();
  void raiseGripper();
  void lowerGripper();
  void activateMotors();
  void deactivateMotors();

  //
  // State Query Functions
  //
  bool gripperMoving();
  bool gripperClosed();
  bool gripperOpen();
  bool liftMoving();
  bool motorsActivated();

 protected:
  ros::NodeHandle n_;

  ros::Subscriber gripper_state_sub_;
  ros::Subscriber motor_state_sub_;

  ros::Publisher gripper_pub_;
  ros::Publisher ptz_pub_;
  ros::Publisher motor_state_pub_;

 public:
  p2os::GripperState gripper_state_;
  p2os::MotorState motor_state_;

  //
  // Constants
  //
  static const int GRIP_OPEN;
  static const int GRIP_CLOSE;
  static const int GRIP_STOP;
  static const int LIFT_UP;
  static const int LIFT_DOWN;
  static const int LIFT_STOP;
  static const int GRIP_STORE;
  static const int GRIP_DEPLOY;
  static const int GRIP_HALT;
  static const int GRIP_PRESS;
  static const int LIFT_CARRY;
  static const unsigned int GRIPPER_OPEN_STATE;
  static const unsigned int GRIPPER_CLOSED_STATE;
  static const unsigned int GRIPPER_MOVING_STATE;
  static const int LIFT_STATIC_STATE;
  static const int LIFT_MOVING_STATE;
  static const int MOTORS_ON;
  static const int MOTORS_OFF;
};
#endif //pioneer_wrapper_h_DEFINED
