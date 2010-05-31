/*
 * teleop_pioneer_keyboard
 * Copyright (c) 2010, Tucker Hermans
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Kevin Watts
// Author: Tucker Hermans

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <p2os_driver/GripperState.h>
#include <p2os_driver/MotorState.h>
#include <p2os_driver/PTZState.h>

// Motor State
#define KEYCODE_P 0x70

// Drive Commands
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

// Gripper
#define KEYCODE_F 0x66
#define KEYCODE_G 0x67
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_B 0x62

// PTZ
#define KEYCODE_I 0x69
#define KEYCODE_K 0x6B
#define KEYCODE_J 0x6A
#define KEYCODE_L 0x6C
#define KEYCODE_U 0x75
#define KEYCODE_O 0x6F
#define KEYCODE_M 0x6D

#define KEYCODE_I_CAP 0x49
#define KEYCODE_K_CAP 0x4B
#define KEYCODE_J_CAP 0x4A
#define KEYCODE_L_CAP 0x4C
#define KEYCODE_U_CAP 0x55
#define KEYCODE_O_CAP 0x4F
#define KEYCODE_M_CAP 0x4D

// Encoded parameters
#define MOTOR_ON_VAL  4
#define MOTOR_OFF_VAL 0 // I'm not sure this is correct...
#define GRIP_OPEN     1
#define GRIP_CLOSE    2
#define GRIP_STOP     3
#define LIFT_UP       4
#define LIFT_DOWN     5
#define LIFT_STOP     6

class TeleopPioneerKeyboard
{
 private:
  double walk_vel_, run_vel_, yaw_rate_, yaw_rate_run_;
  double pan_rate_, tilt_rate_, zoom_rate_;
  double fast_pan_rate_, fast_tilt_rate_, fast_zoom_rate_;
  geometry_msgs::Twist cmd_vel_;
  p2os_driver::MotorState motor_cmd_;
  p2os_driver::GripperState grip_cmd_;
  p2os_driver::PTZState ptz_cmd_;

  ros::NodeHandle n_;
  ros::Publisher motor_pub_;
  ros::Publisher vel_pub_;
  ros::Publisher grip_pub_;
  ros::Publisher ptz_pub_;

 public:
  void init()
  {
    cmd_vel_.linear.x = cmd_vel_.linear.y = cmd_vel_.angular.z = 0;
    ptz_cmd_.pan = ptz_cmd_.tilt = ptz_cmd_.zoom = 0;
    ptz_cmd_.relative = false;
    grip_cmd_.grip.state = grip_cmd_.lift.state = 0;
    motor_cmd_.state = MOTOR_OFF_VAL;

    motor_pub_ = n_.advertise<p2os_driver::MotorState>("motor_state_cmd", 1);
    vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    grip_pub_ = n_.advertise<p2os_driver::GripperState>("gripper_cmd", 1);
    ptz_pub_ = n_.advertise<p2os_driver::PTZState>("ptz_cmd", 1);

    ros::NodeHandle n_private("~");
    n_private.param("walk_vel", walk_vel_, 0.5);
    n_private.param("run_vel", run_vel_, 1.0);
    n_private.param("yaw_rate", yaw_rate_, 1.0);
    n_private.param("yaw_run_rate", yaw_rate_run_, 1.5);
    n_private.param("pan_rate", pan_rate_, 5.0);
    n_private.param("tilt_rate", tilt_rate_, 5.0);
    n_private.param("zoom_rate", zoom_rate_, 196.0);
    n_private.param("fast_pan_rate", fast_pan_rate_, 20.0);
    n_private.param("fast_tilt_rate", fast_tilt_rate_, 20.0);
    n_private.param("fast_zoom_rate", fast_zoom_rate_, 392.0);
  }

  ~TeleopPioneerKeyboard()   { }
  void keyboardLoop();

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_base_keyboard");

  TeleopPioneerKeyboard tpk;
  tpk.init();

  signal(SIGINT,quit);

  tpk.keyboardLoop();

  return(0);
}

void TeleopPioneerKeyboard::keyboardLoop()
{
  char c;
  bool motor_dirty = false;
  bool vel_dirty = false;
  bool gripper_dirty = false;
  bool ptz_dirty = false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WS' to translate");
  puts("Use 'AD' to yaw");
  puts("Press 'Shift' to run");
  puts("Use FG to control the gripper");
  puts("Use RT to control the lift");
  puts("Use B to stop the lift and gripper");
  puts("Use JL to pan");
  puts("Use KI to tilt");
  puts("Use UO to zoom");

  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    // Clear the previous commands
    cmd_vel_.linear.x = cmd_vel_.linear.y = cmd_vel_.angular.z = 0;
    ptz_cmd_.pan = ptz_cmd_.tilt = ptz_cmd_.zoom = 0;

    motor_dirty = false;
    //vel_dirty = false;
    gripper_dirty = false;
    ptz_dirty = false;

    switch(c)
    {
      // Motor state
      case KEYCODE_P:
        motor_cmd_.state = MOTOR_ON_VAL;
        motor_dirty = true;
        break;

        // Walking
      case KEYCODE_W:
        cmd_vel_.linear.x = walk_vel_;
        vel_dirty = true;
        break;
      case KEYCODE_S:
        cmd_vel_.linear.x = - walk_vel_;
        vel_dirty = true;
        break;
      case KEYCODE_A:
        cmd_vel_.angular.z = yaw_rate_;
        vel_dirty = true;
        break;
      case KEYCODE_D:
        cmd_vel_.angular.z = - yaw_rate_;
        vel_dirty = true;
        break;

        // Running
      case KEYCODE_W_CAP:
        cmd_vel_.linear.x = run_vel_;
        vel_dirty = true;
        break;
      case KEYCODE_S_CAP:
        cmd_vel_.linear.x = - run_vel_;
        vel_dirty = true;
        break;
      case KEYCODE_A_CAP:
        cmd_vel_.angular.z = yaw_rate_run_;
        vel_dirty = true;
        break;
      case KEYCODE_D_CAP:
        cmd_vel_.angular.z = - yaw_rate_run_;
        vel_dirty = true;
        break;

        // Gripper
      case KEYCODE_F:
        grip_cmd_.grip.state = GRIP_OPEN;
        gripper_dirty = true;
        break;
      case KEYCODE_G:
        grip_cmd_.grip.state = GRIP_CLOSE;
        gripper_dirty = true;
        break;
      case KEYCODE_R:
        grip_cmd_.lift.state = LIFT_DOWN;
        gripper_dirty = true;
        break;
      case KEYCODE_T:
        grip_cmd_.lift.state = LIFT_UP;
        gripper_dirty = true;
        break;
      case KEYCODE_B:
        grip_cmd_.grip.state = GRIP_STOP;
        grip_cmd_.lift.state = LIFT_STOP;
        gripper_dirty = true;
        break;

        // PTZ
      case KEYCODE_I:
        ptz_cmd_.tilt = tilt_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_K:
        ptz_cmd_.tilt = -tilt_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_J:
        ptz_cmd_.pan = -pan_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_L:
        ptz_cmd_.pan = pan_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_U:
        ptz_cmd_.zoom = -zoom_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_O:
        ptz_cmd_.zoom = zoom_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_I_CAP:
        ptz_cmd_.tilt = fast_tilt_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_K_CAP:
        ptz_cmd_.tilt = -fast_tilt_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_J_CAP:
        ptz_cmd_.pan = -fast_pan_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_L_CAP:
        ptz_cmd_.pan = fast_pan_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_U_CAP:
        ptz_cmd_.zoom = -fast_zoom_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_O_CAP:
        ptz_cmd_.zoom = fast_zoom_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_M:
        ptz_cmd_.pan = 0;
        ptz_cmd_.tilt = 0;
        ptz_cmd_.zoom = 0;
        ptz_cmd_.relative = false;
        ptz_dirty = true;
        break;
      default:
        break;
    }

    if (motor_dirty)
    {
      motor_pub_.publish(motor_cmd_);
    }

    if (vel_dirty)
    {
      vel_pub_.publish(cmd_vel_);
    }

    if (gripper_dirty)
    {
      grip_pub_.publish(grip_cmd_);
    }

    if (ptz_dirty)
    {
      ptz_pub_.publish(ptz_cmd_);
    }

  }
}
