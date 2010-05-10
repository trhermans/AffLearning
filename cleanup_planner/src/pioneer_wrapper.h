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
  static const int GRIPPER_OPEN_STATE;
  static const int GRIPPER_CLOSED_STATE;
  static const int GRIPPER_MOVING_STATE;
  static const int LIFT_STATIC_STATE;
  static const int LIFT_MOVING_STATE;
  static const int MOTORS_ON;
  static const int MOTORS_OFF;
};
