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
#ifndef overhead_tracking_h_DEFINED
#define overhead_tracking_h_DEFINED

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <vector>
#include <string>
#include "overhead_tracking/CleanupObjectArray.h"
#include "overhead_tracking/CleanupObject.h"
#include "geometry_msgs/Pose2D.h"

//
// Class to calculate and store color histograms of images and regions of images
//
class RGBHistogram
{
 public:
  // Constructors and destructors
  RGBHistogram(int r_bins=16, int g_bins=16, int b_bins=16);
  virtual ~RGBHistogram();

  // Core functions
  void createHist(cv::Mat img);
  void createHist(cv::Mat img, std::vector<cv::Point> contour);
  double compareHists(RGBHistogram compare_hist);

  cv::MatND* getHist()
  {
    return &hist_;
  }

 protected:
  int r_bins_;
  int g_bins_;
  int b_bins_;
  cv::MatND hist_;
};

//
// Class to store all relevant information to a specific object being tracked.
//
class OverheadVisualObject
{
 public:
  overhead_tracking::CleanupObject state;
  RGBHistogram color;
  std::vector<cv::Point> contour;
  cv::Moments moments;
  OverheadVisualObject() {}
};

//
// Class to perform the tracking overtime
//
class OverheadTracker
{
 public:
  // Constructors
  OverheadTracker(std::string window_name);

  // Core funcitons
  void updateDisplay(cv::Mat update_img,
                     std::vector<std::vector<cv::Point> > object_contours);
  void initTracks(std::vector<std::vector<cv::Point> >& object_contours,
                  std::vector<cv::Moments>& object_moments);
  void initRobotTrack(std::vector<cv::Point>& robot_contour,
                      cv::Moments& robot_moments);
  void updateTracks(std::vector<std::vector<cv::Point> >& object_contours,
                    std::vector<cv::Moments>& object_moments);
  void updateRobotTrack(std::vector<cv::Point>& robot_contour,
                        cv::Moments& robot_moments);

  // User IO Methods
  static void onWindowClick(int event, int x, int y, int flags, void* param);
  void onKeyCallback(char c);
  void addBoundaryPoint(cv::Point pt);

  // Getters and setters
  bool addingContour() const
  {
    return drawing_boundary_;
  }

  std::vector<OverheadVisualObject> getTrackedObjects()
  {
    return tracked_objects_;
  }

  geometry_msgs::Pose2D getRobotPose()
  {
    return tracked_robot_.state.pose;
  }
  // Members
 protected:
  // Tracking members
  std::vector<OverheadVisualObject> tracked_objects_;
  OverheadVisualObject tracked_robot_;

  // User IO members
  std::vector<std::vector<cv::Point> > boundary_contours_;
  std::vector<cv::Point> working_boundary_;
  cv::Scalar object_center_color_;
  cv::Scalar object_contour_color_;
  cv::Scalar robot_contour_color_;
  cv::Scalar boundary_color_;
  std::string window_name_;
  bool drawing_boundary_;

  // Tracking and state parameters
  int min_contour_size_;
  bool tracking_;
  bool initialized_;
  unsigned long run_count_;

  // Constants
 public:
  static const int MAX_MIN_SIZE;
  static const unsigned int MIN_NUM_CONTOUR_POINTS;
  static const char DRAW_BOUNDARY_KEY;
  static const char CLEAR_BOUNDARIES_KEY;
  static const char CLEAR_WORKING_BOUNDARY_KEY;
  static const char TOGGLE_TRACKING_KEY;
};
#endif // overhead_tracking_h_DEFINED
