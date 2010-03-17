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
#include <opencv/highgui.h>
#include "overhead_tracking.h"
#include <ros/ros.h>

using namespace cv;
using std::vector;
using std::string;
using overhead_tracking::CleanupObjectArray;
using overhead_tracking::CleanupObject;

inline int sign(double x)
{
  if(x < 0)
    return -1;
  return 1;
}

// Constants
const int OverheadTracker::MAX_MIN_SIZE = 1000;
const unsigned int OverheadTracker::MIN_NUM_CONTOUR_POINTS = 3;
const char OverheadTracker::DRAW_BOUNDARY_KEY = 'd';
const char OverheadTracker::CLEAR_BOUNDARIES_KEY = 'k';
const char OverheadTracker::CLEAR_WORKING_BOUNDARY_KEY = 'w';
const char OverheadTracker::TOGGLE_TRACKING_KEY = 't';

OverheadTracker::OverheadTracker(String window_name) :
    object_center_color_(0, 255, 0),
    object_contour_color_(0, 0, 255),
    robot_contour_color_(255, 0, 0),
    boundary_color_(0, 255, 255),
    window_name_(window_name), drawing_boundary_(false),
    min_contour_size_(0), tracking_(false), initialized_(false), run_count_(0)
{
  boundary_contours_.clear();
  working_boundary_.clear();

  // Create a HighGUI window for displaying and controlling the tracker
  namedWindow(window_name_);
  createTrackbar("Min Size", window_name_, &min_contour_size_,
                 MAX_MIN_SIZE);
  cvSetMouseCallback(window_name_.c_str(), onWindowClick, this);
  tracked_robot_.state.x = 0;
  tracked_robot_.state.y = 0;
  tracked_robot_.state.dx = 0;
  tracked_robot_.state.dy = 0;
}

/**
 * Initialize the object tracks
 *
 * @param object_moments Associated moments
 */
void OverheadTracker::initTracks(vector<vector<Point> >& object_contours,
                                 std::vector<cv::Moments>& object_moments)
{
  initialized_ = true;
  tracked_objects_.clear();
  for (unsigned int i = 0; i < object_moments.size(); ++i)
  {
    OverheadVisualObject obj;
    obj.state.x = object_moments[i].m10 / object_moments[i].m00;
    obj.state.y = object_moments[i].m01 / object_moments[i].m00;
    obj.state.dx = 0;
    obj.state.dy = 0;
    obj.moments = object_moments[i];
    tracked_objects_.push_back(obj);
  }
}

void OverheadTracker::initRobotTrack(vector<Point>& robot_contour,
                                     Moments& robot_moments)
{
  tracked_robot_.state.x = robot_moments.m10 / robot_moments.m00;
  tracked_robot_.state.y = robot_moments.m01 / robot_moments.m00;

  tracked_robot_.state.dx = 0;
  tracked_robot_.state.dy = 0;
  tracked_robot_.contour = robot_contour;
}

/**
 * Update the overhead tracker display with the most recent image and contours.
 * Also controls functionality for interacting with the tracker.
 *
 * @param update_img_raw Image to update
 * @param contours Object contours to update
 */
void OverheadTracker::updateDisplay(Mat update_img_raw,
                                    vector<vector<Point> > contours)
{
  // Copy the image so that we don't get lots of old object boundaries drawn
  // when the update image is slow to refresh
  Mat update_img;
  update_img_raw.copyTo(update_img);
  vector<Moments> object_moments;

  // Update the object tracks before drawing the updates on the display
  if (tracking_)
  {
    double max_size = 0;
    int max_idx = -1;
    // Iterate through the contours, removing those with area less than min_size
    for (unsigned int i = 0; i < contours.size();)
    {
      Moments m;
      m = moments(contours[i]);
      if (m.m00 < min_contour_size_)
      {
        contours.erase(contours.begin()+i);
      }
      else
      {
        if(m.m00 > max_size)
        {
          max_size = m.m00;
          max_idx = i;
        }
        object_moments.push_back(m);
        ++i;
      }
    }


    if (!initialized_)
    {
      ROS_DEBUG("Initializing tracking on run %lu", run_count_);
      initTracks(contours, object_moments);
      if (max_idx != -1)
      {
        initRobotTrack(contours[max_idx], object_moments[max_idx]);
      }
    }
    else
    {
      ROS_DEBUG("Updating on run %lu", run_count_);
      updateTracks(contours, object_moments);
      if (max_idx != -1)
      {
        updateRobotTrack(contours[max_idx], object_moments[max_idx]);
      }
    }
    // Remove Robot from set
    if( max_idx != -1)
    {
      object_moments.erase(object_moments.begin() + max_idx);
      contours.erase(contours.begin() + max_idx);
    }

    // Draw contour centers
    for (unsigned int i = 0; i < tracked_objects_.size(); ++i)
    {
      Point center(tracked_objects_[i].state.x,
                   tracked_objects_[i].state.y);
      circle(update_img, center, 4, object_center_color_, 2);
    }

    // Draw contours
    if (contours.size() > 0)
    {
      drawContours(update_img, contours, -1, object_contour_color_, 2);
    }

    if (tracked_robot_.state.x != 0 || tracked_robot_.state.y != 0)
    {
      vector<vector<Point> > robot_contours;
      robot_contours.push_back(tracked_robot_.contour);
      drawContours(update_img, robot_contours, -1, robot_contour_color_, 2);
      Point center(tracked_robot_.state.x, tracked_robot_.state.y);
      circle(update_img, center, 4, robot_contour_color_, 2);
      // TODO: Draw robot orientation...
    }
  }

  // Draw user defined boundaries
  if (boundary_contours_.size() > 0)
  {
    drawContours(update_img, boundary_contours_, -1,
                 boundary_color_, 2);
  }

  // Now show the updated image
  imshow(window_name_, update_img);
  char c = waitKey(3);

  onKeyCallback(c);
  run_count_++;
}


/**
 * Performs the matching between the previous frame contours and those found in
 * the current frame.
 *
 * @param object_contours The object contours for the current frame
 * @param object_moments The associated moments for the current frame
 */
void OverheadTracker::updateTracks(vector<vector<Point> >& object_contours,
                                   std::vector<cv::Moments>& object_moments)
{
  // Match new regions to previous ones
  for (unsigned int i = 0; i < object_moments.size() &&
           tracked_objects_.size() > 0; ++i)
  {
    double min_space_dist = DBL_MAX;
    int min_space_idx = -1;
    ROS_DEBUG("Scores for contour %u", i);
    for (unsigned int j = 0; j < tracked_objects_.size(); ++j)
    {
      double dX = tracked_objects_[i].state.x - (object_moments[j].m10 /
                                                 object_moments[j].m00);
      double dY = tracked_objects_[i].state.y - (object_moments[j].m01 /
                                                 object_moments[j].m00);
      double space_dist = sqrt(dX*dX + dY*dY);
      if (space_dist < min_space_dist)
      {
        min_space_dist = space_dist;
        min_space_idx = j;
      }
    }
    // We have found the mimimum score, update our instances...
    ROS_DEBUG("Min spacial distnace is %le at point %u\n",
              min_space_dist, min_space_idx);
  }

  // Now that we've matched tracks, clear the old ones and replace them with the
  // new ones
  tracked_objects_.clear();
  for (unsigned int i = 0; i < object_moments.size(); ++i)
  {
    OverheadVisualObject obj;
    obj.state.x = object_moments[i].m10 / object_moments[i].m00;
    obj.state.y = object_moments[i].m01 / object_moments[i].m00;
    obj.state.dx = 0;
    obj.state.dy = 0;
    obj.moments = object_moments[i];
    tracked_objects_.push_back(obj);
  }

}


void OverheadTracker::updateRobotTrack(vector<Point>& robot_contour,
                                       Moments& robot_moments)
{
  double newX = robot_moments.m10 / robot_moments.m00;
  double newY = robot_moments.m01 / robot_moments.m00;
  tracked_robot_.state.dx = tracked_robot_.state.x - newX;
  tracked_robot_.state.dy = tracked_robot_.state.y - newY;

  tracked_robot_.state.x = newX;
  tracked_robot_.state.y = newY;

  tracked_robot_.contour = robot_contour;
}

//
// User IO Methods
//

/**
 * Add a point to the currently building boundary contour
 *
 * @param pt The point to be added
 */
void OverheadTracker::addBoundaryPoint(Point pt)
{
  if (drawing_boundary_)
  {
    working_boundary_.push_back(pt);
  }
}

/**
 * Perform the requist action dependent on the user input key
 *
 * @param c The input key
 */
void OverheadTracker::onKeyCallback(char c)
{
  // Perform actions corresponding to key presses
  if (c == DRAW_BOUNDARY_KEY)
  {
    if (drawing_boundary_)
    {
      ROS_INFO("Stopping boundary drawing.");
      // Accept the current boundary as a contour
      if (working_boundary_.size() >= MIN_NUM_CONTOUR_POINTS) {
        boundary_contours_.push_back(working_boundary_);
        ROS_INFO("Saved boundary.");
      }
      else
      {
        ROS_INFO("Boundary too small, discarding.");
      }
      working_boundary_.clear();
    }
    else
    {
      ROS_INFO("Begining to draw boundary.");
    }
    // Toggle drawing boundary on and off
    drawing_boundary_ = ! drawing_boundary_;
  }

  if (c == CLEAR_BOUNDARIES_KEY)
  {
    boundary_contours_.clear();
    ROS_INFO("Cleared boundaries.");
  }

  if (c == CLEAR_WORKING_BOUNDARY_KEY)
  {
    working_boundary_.clear();
    ROS_INFO("Cleared working boundary.");
  }

  if (c == TOGGLE_TRACKING_KEY)
  {
    if (!tracking_)
    {
      tracking_ = true;
      ROS_INFO("Tracking Began.");
    }
    else
    {
      tracking_ = false;
      initialized_ = false;
      ROS_INFO("Tracking ended.");
    }
  }
}

/**
 * Callback method for a mouse click from opencv
 *
 * @param event The mouse event
 * @param x x-location of the pointer
 * @param y y-location of the pointer
 * @param flags associated open CV flags
 * @param param Pointer to the calling OverheadTracker instances
 */
void OverheadTracker::onWindowClick(int event, int x, int y,
                                    int flags, void* param)
{
  OverheadTracker * tracker;
  tracker = reinterpret_cast<OverheadTracker*>(param);

  if (!tracker->addingContour())
  {
    return;
  }

  Point pt(x,y);

  switch(event)
  {
    case CV_EVENT_LBUTTONDOWN:
      break;
    case CV_EVENT_RBUTTONDOWN:
      break;
    case CV_EVENT_MBUTTONDOWN:
      break;
    case CV_EVENT_LBUTTONUP:
      break;
    case CV_EVENT_RBUTTONUP:
      break;
    case CV_EVENT_MBUTTONUP:
      break;
    case CV_EVENT_RBUTTONDBLCLK:
      break;
    case CV_EVENT_LBUTTONDBLCLK:
      // Add the current point to the current contour
      tracker->addBoundaryPoint(pt);
      break;
    case CV_EVENT_MBUTTONDBLCLK:
      break;
    case CV_EVENT_MOUSEMOVE:
      break;
    default:
      break;
  }
}

//
// Color Histogram Stuff
//

RGBHistogram::RGBHistogram(int r_bins, int g_bins, int b_bins) :
    r_bins_(r_bins), g_bins_(g_bins), b_bins_(b_bins)
{
}

RGBHistogram::~RGBHistogram()
{
}

void RGBHistogram::createHist(Mat img)
{
  int hist_size[] = {r_bins_, g_bins_, b_bins_};
  float range[] = {0, 256};
  const float* ranges[] = {range, range, range};
  int channels[] = {0,1,2};

  calcHist( &img, 1, static_cast<int*>(channels), Mat(), // do not use mask
            hist_, 2, static_cast<int*>(hist_size),
            static_cast<const float**>(ranges), true, false );
}

void RGBHistogram::createHist(Mat img, vector<Point> contour)
{
  int hist_size[] = {r_bins_, g_bins_, b_bins_};
  float range[] = {0, 256};
  const float* ranges[] = {range, range, range};
  int channels[] = {0,1,2};

  // Build a mask from the contour
  Mat mask(img.size(), CV_8UC1, Scalar(0));
  Point* contour_p;
  contour_p = contour.get_allocator().allocate(contour.size());
  fillConvexPoly(mask, contour_p, contour.size(), Scalar(1));

  calcHist( &img, 1, static_cast<int*>(channels), mask,
            hist_, 2, static_cast<int*>(hist_size),
            static_cast<const float**>(ranges), true, false);

}

double RGBHistogram::compareHists(RGBHistogram compare_hist)
{
  // CV_COMP_CORREL - Correlation
  // CV_COMP_CHISQR - Chi-Square
  // CV_COMP_INTERSECT - Intersection
  // CV_COMP_BHATTACHARYYA - Bhattacharyya distance
  return compareHist(hist_, *compare_hist.getHist(), CV_COMP_INTERSECT);
}
