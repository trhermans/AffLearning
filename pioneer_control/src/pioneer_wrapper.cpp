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

#include "pioneer_wrapper.h"

//
// Assignment of class constants
//
const int PioneerWrapper::GRIP_OPEN   = 1;
const int PioneerWrapper::GRIP_CLOSE  = 2;
const int PioneerWrapper::GRIP_STOP   = 3;
const int PioneerWrapper::LIFT_UP     = 4;
const int PioneerWrapper::LIFT_DOWN   = 5;
const int PioneerWrapper::LIFT_STOP   = 6;
const int PioneerWrapper::GRIP_STORE  = 7;
const int PioneerWrapper::GRIP_DEPLOY = 8;
const int PioneerWrapper::GRIP_HALT   = 15;
const int PioneerWrapper::GRIP_PRESS  = 16;
const int PioneerWrapper::LIFT_CARRY  = 17;

const unsigned int PioneerWrapper::GRIPPER_OPEN_STATE   = 1;
const unsigned int PioneerWrapper::GRIPPER_CLOSED_STATE = 2;
const unsigned int PioneerWrapper::GRIPPER_MOVING_STATE = 3;
const int PioneerWrapper::LIFT_STATIC_STATE    = 1;
const int PioneerWrapper::LIFT_MOVING_STATE    = 2;

const int PioneerWrapper::MOTORS_ON  = 4;
const int PioneerWrapper::MOTORS_OFF = 0;

//
// Constructors
//
PioneerWrapper::PioneerWrapper(ros::NodeHandle &n) :
    n_(n)
{
  gripper_state_sub_ = n_.subscribe("gripper_state", 1,
                                    &PioneerWrapper::gripperStateCallback,
                                    this);
  motor_state_sub_ = n_.subscribe("motor_state", 1,
                                  &PioneerWrapper::motorStateCallback,
                                  this);

  gripper_pub_ = n_.advertise<p2os::GripperState>("gripper_cmd",1000);
  motor_state_pub_ = n_.advertise<p2os::MotorState>("motor_state_cmd",1000);
  ptz_pub_ = n_.advertise<p2os::PTZState>("ptz_cmd",1000);
}

//
// Callback Functions
//

void PioneerWrapper::gripperStateCallback(const p2os::GripperStateConstPtr &msg)
{
  gripper_state_.grip.state = msg->grip.state;
  gripper_state_.grip.dir = msg->grip.dir;

  gripper_state_.grip.inner_beam = msg->grip.inner_beam;
  gripper_state_.grip.outer_beam = msg->grip.outer_beam;

  gripper_state_.grip.left_contact = msg->grip.left_contact;
  gripper_state_.grip.right_contact = msg->grip.right_contact;

  gripper_state_.lift.state = msg->lift.state;
  gripper_state_.lift.dir = msg->lift.dir;
  gripper_state_.lift.position = msg->lift.position;

}

void PioneerWrapper::motorStateCallback(const p2os::MotorStateConstPtr &msg)
{
  motor_state_.state = msg->state;
  if (!motorsActivated())
    activateMotors();
}

//
// Mid-level gripper behaviors
//
bool PioneerWrapper::grabObject()
{
  // Assume in line with object and open grippers
  if (! gripperMoving())
  {
    if( gripperOpen())
      closeGripper();
    else
      return true;
  }
  return false;
}

bool PioneerWrapper::releaseObject()
{
  // Assume gripper is closed and down
  if (! gripperMoving())
  {
    if( gripperClosed())
      openGripper();
    else
      return true;
  }
  return false;
}

bool PioneerWrapper::pickupObject()
{
  if (grabObject())
  {
    if (! liftMoving())
    {
      if (gripper_state_.lift.position < 1.0)
        raiseGripper();
      else
        return true;
    }
  }
  return false;
}

bool PioneerWrapper::putDownObject()
{
  if (! liftMoving())
  {
    if (gripper_state_.lift.position > 0.0)
      lowerGripper();
    else if (releaseObject())
      return true;
  }
  return false;
}


//
// Low Level Gripper Commands
//
void PioneerWrapper::deployGripper()
{
  p2os::GripperState g;
  g.grip.state = GRIP_OPEN;
  g.lift.state = LIFT_DOWN;
  gripper_pub_.publish(g);
}

void PioneerWrapper::storeGripper()
{
  p2os::GripperState g;
  g.grip.state = GRIP_CLOSE;
  g.lift.state = LIFT_UP;
  gripper_pub_.publish(g);
}

void PioneerWrapper::openGripper()
{
  p2os::GripperState g;
  g.grip.state = GRIP_OPEN;
  gripper_pub_.publish(g);
}

void PioneerWrapper::closeGripper()
{
  p2os::GripperState g;
  g.grip.state = GRIP_CLOSE;
  gripper_pub_.publish(g);
}

void PioneerWrapper::raiseGripper()
{
  p2os::GripperState g;
  g.lift.state = LIFT_UP;
  gripper_pub_.publish(g);
}

void PioneerWrapper::lowerGripper()
{
  p2os::GripperState g;
  g.lift.state = LIFT_UP;
  gripper_pub_.publish(g);
}

//
// Motor Commands
//
void PioneerWrapper::activateMotors()
{
  p2os::MotorState m;
  m.state = MOTORS_ON;
  motor_state_pub_.publish(m);
}

void PioneerWrapper::deactivateMotors()
{
  p2os::MotorState m;
  m.state = MOTORS_OFF;
  motor_state_pub_.publish(m);
}


//
// State Query Functions
//
bool PioneerWrapper::gripperMoving()
{
  return (gripper_state_.grip.state == GRIPPER_MOVING_STATE);
}

bool PioneerWrapper::gripperClosed()
{
  return (gripper_state_.grip.state == GRIPPER_CLOSED_STATE);
}

bool PioneerWrapper::gripperOpen()
{
  return (gripper_state_.grip.state == GRIPPER_OPEN_STATE);
}

bool PioneerWrapper::liftMoving()
{
  return (gripper_state_.lift.state == LIFT_MOVING_STATE);
}

bool PioneerWrapper::motorsActivated()
{
  return (motor_state_.state == MOTORS_ON);
}
