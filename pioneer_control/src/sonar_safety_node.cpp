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
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "p2os/SonarArray.h"

/**
 * @file   sonar_safety_node.cpp
 * @author Tucker Hermans <thermans@cc.gatech.edu>
 * @date   Wed Apr 21 18:02:52 2010
 *
 * @brief  Class to preempt running into things by using sonar
 *
 */
class SonarSafetyNode
{
 public:
  SonarSafetyNode(ros::NodeHandle n) :
      n_(n), safe_dist_(0.3), forward_detection_(false),
      lateral_detection_(false)
  {
    sonar_sub_ = n_.subscribe("sonar", 1, &SonarSafetyNode::sonarCallback,
                              this);
    raw_cmd_sub_ = n_.subscribe("raw_cmd_vel", 1,
                                &SonarSafetyNode::rawCmdCallback, this);
    safe_cmd_pub_ = n_.advertise<geometry_msgs::Twist>("safe_cmd_vel", 1000);
  }

  void updateSafeVel()
  {
    geometry_msgs::Twist safe_cmd_vel;
    if (forward_detection_ && (raw_cmd_vel_.linear.x > 0.0))
    {
      safe_cmd_vel.linear.x = 0.0;
    }
    else
    {
      safe_cmd_vel.linear.x = raw_cmd_vel_.linear.x;

    }
    if (lateral_detection_)
    {
      safe_cmd_vel.angular.z = 0.0;
    }
    else
    {
      safe_cmd_vel.angular.z = raw_cmd_vel_.angular.z;
    }
    safe_cmd_pub_.publish(safe_cmd_vel);
  }

  /**
   * Callback method for the sonar array topic.
   * Will make sure we send a new command if needed
   *
   * @param sonar_data Latest msg on the "/sonar" topic
   */
  void sonarCallback(const p2os::SonarArrayConstPtr &sonar_data)
  {
    bool prev_forward_detection = forward_detection_;
    bool prev_lateral_detection = lateral_detection_;

    forward_detection_ = false;

    for (unsigned int i = 1; i < 7; ++i)
    {
      if (sonar_data->ranges[i] < safe_dist_)
      {
        forward_detection_ = true;
      }
    }

    lateral_detection_ = (false &&
                          (sonar_data->ranges[0] < safe_dist_ ||
                           sonar_data->ranges[7] < safe_dist_ ));

    // Change our safe velocity command if the detection state changes
    if (prev_forward_detection != forward_detection_ ||
        prev_lateral_detection != lateral_detection_)
      updateSafeVel();
  }

  /**
   * Callback method for the raw velocity command. Updates are current desired
   * velocity and reports when a velocity command update has been made.
   *
   * @param raw_cmd Raw velocity command message
   */
  void rawCmdCallback(const geometry_msgs::TwistConstPtr &raw_cmd)
  {
    raw_cmd_vel_.linear.x = raw_cmd->linear.x;
    raw_cmd_vel_.angular.z = raw_cmd->angular.z;

    // Report the updated value
    updateSafeVel();
  }


  void spin()
  {
    while(n_.ok())
    {
      ros::spinOnce();
    }
  }

 protected:
  ros::NodeHandle n_;
  ros::Subscriber sonar_sub_;
  ros::Subscriber raw_cmd_sub_;
  ros::Publisher safe_cmd_pub_;
  geometry_msgs::Twist raw_cmd_vel_;
  float safe_dist_;
  bool forward_detection_;
  bool lateral_detection_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sonar_safety_node");
  ros::NodeHandle n;
  SonarSafetyNode ssn(n);
  ssn.spin();
}
