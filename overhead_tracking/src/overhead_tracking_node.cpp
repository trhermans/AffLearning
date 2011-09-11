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
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

#include "overhead_tracking.h"
#include "bg_subtract.h"
#include "overhead_tracking/CleanupObjectArray.h"
#include "overhead_tracking/CleanupObject.h"
#include "overhead_tracking/CleanupZoneArray.h"
#include "overhead_tracking/Pose2DStamped.h"
#include "overhead_tracking/ClearData.h"
#include "geometry_msgs/Pose2D.h"

// #define DOWNSAMPLE_OVERHEAD_IMAGE

using geometry_msgs::Pose2D;
using overhead_tracking::Pose2DStamped;
using overhead_tracking::CleanupObjectArray;
using overhead_tracking::CleanupZoneArray;
using overhead_tracking::ClearData;

class OverheadTrackingNode
{
 public:
  // Constructors and Destructors
  OverheadTrackingNode(ros::NodeHandle &n) :
      n_(n), it_(n), bg_gui_(), tracker_("Contour Window", &bg_gui_.bg_sub_)
  {
    image_sub_ = it_.subscribe("image_topic", 1,
                               &OverheadTrackingNode::imageCallback, this);
    clear_goal_sub_ = n_.subscribe("clear_goal_pose", 1,
                                   &OverheadTrackingNode::clearGoalCallback,
                                   this);

    pose_pub_ = n_.advertise<Pose2DStamped>("robot_pose", 1000);
    object_pub_ = n_.advertise<CleanupObjectArray>("cleanup_objects", 1000);
    goal_pub_ = n_.advertise<Pose2DStamped>("goal_pose", 1000);
    cleanup_zone_pub_ = n_.advertise<CleanupZoneArray>("cleanup_zones", 1000);
  }

  // Publish and Subscribe methods

  /**
   * Callback method to the image transport subscriber. Currently only saves a
   * copy of the most recent image internally to the node for future use.
   *
   * @param msg_ptr Most recent image off of the topic
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
  {
    cv::Mat fg_mat;
    fg_mat = bridge_.imgMsgToCv(msg_ptr);

    // Save a copy of the image
    // cv::Mat init_copy;
#ifdef DOWNSAMPLE_OVERHEAD_IMAGE
    fg_mat.copyTo(init_copy);
    cv::pyrDown(init_copy, current_img_);
#else
    fg_mat.copyTo(current_img_);
#endif // DOWNSAMPLE_OVERHEAD_IMAGE
    bg_gui_.updateDisplay(current_img_);
    contours_ = bg_gui_.bg_sub_.getContours();

    // Update our contour image
    tracker_.updateDisplay(current_img_, contours_);
    publishData();
  }

  void clearGoalCallback(const ClearData msg)
  {
    if (msg.clear)
      tracker_.clearWaypoints();
  }

  void publishData()
  {
    ros::Time pub_time = ros::Time::now();
    Pose2D tracker_robot_pose;
    tracker_robot_pose = tracker_.getRobotPose();

    Pose2DStamped robot_pose;
    robot_pose.header.stamp = pub_time;
    robot_pose.pose.x = tracker_robot_pose.x;
    robot_pose.pose.y = tracker_robot_pose.y;
    robot_pose.pose.theta = tracker_robot_pose.theta;

    pose_pub_.publish(robot_pose);

    CleanupObjectArray cleanup_objs;
    cleanup_objs = tracker_.getCleanupObjects();
    cleanup_objs.header.stamp = pub_time;

    object_pub_.publish(cleanup_objs);

    if (tracker_.newCleanupZone())
    {
      CleanupZoneArray cleanup_zones;
      cleanup_zones = tracker_.getCleanupZones();
      cleanup_zones.header.stamp = pub_time;

      cleanup_zone_pub_.publish(cleanup_zones);
    }
    if (tracker_.newGoalPose())
    {
      Pose2D tracker_goal_pose = tracker_.getGoalPose();
      Pose2DStamped goal_pose;

      goal_pose.header.stamp = pub_time;
      goal_pose.pose.x = tracker_goal_pose.x;
      goal_pose.pose.y = tracker_goal_pose.y;
      goal_pose.pose.theta = tracker_goal_pose.theta;

      goal_pub_.publish(goal_pose);
    }
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
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
  ros::Subscriber clear_goal_sub_;

  ros::Publisher pose_pub_, object_pub_, goal_pub_, cleanup_zone_pub_;
  BgSubtractGUI bg_gui_;
  OverheadTracker tracker_;
  cv::Mat current_img_;
  std::vector<std::vector<cv::Point> > contours_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "overhead_tracking");
  ros::NodeHandle n;
  OverheadTrackingNode overhead_node(n);
  overhead_node.spin();
}

