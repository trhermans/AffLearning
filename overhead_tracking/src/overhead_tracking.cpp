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
#include "overhead_tracking.h"
#include <ros/ros.h>
#include <opencv/highgui.h>
#include <map>
#include <sstream>

//#define SAVE_IMAGES
#define SHOW_OBJ_IDS
using namespace cv;
using std::vector;
using std::string;
using std::multimap;
using std::pair;
using std::stringstream;
using overhead_tracking::CleanupObjectArray;
using overhead_tracking::CleanupObject;
using geometry_msgs::Pose2D;

// Constants
// Bg keys used are 'c', 'e', 'a'
const int OverheadTracker::MAX_MIN_SIZE = 1000;
const unsigned int OverheadTracker::MIN_NUM_CONTOUR_POINTS = 3;
const char OverheadTracker::DRAW_BOUNDARY_KEY = 'd';
const char OverheadTracker::CLEAR_BOUNDARIES_KEY = 'k';
const char OverheadTracker::CLEAR_WORKING_BOUNDARY_KEY = 'w';
const char OverheadTracker::TOGGLE_TRACKING_KEY = 't';
const char OverheadTracker::INIT_ORIENTATION_KEY = 'o';
const char OverheadTracker::PAUSE_TRACKING_KEY = 'p';
const char OverheadTracker::CLEAR_WAYPOINT_KEY = 'z';
const double OverheadTracker::MIN_DIST_THRESH = 500;
const int OverheadTracker::CAMERA_MAX_X = 1280; // Pixels
const int OverheadTracker::CAMERA_MAX_Y = 960; // Pixels
const double OverheadTracker::WORLD_MAX_X = 2.70; // Meters
const double OverheadTracker::WORLD_MAX_Y = 2.05; // Meters

inline int sign(double x)
{
  if (x < 0)
    return -1;
  else if (x > 0)
    return 1;
  else
    return 0;
}

OverheadTracker::OverheadTracker(string window_name, BgSubtract* bg) :
    bg_(bg),
    // IO Members
    object_center_color_(0, 255, 0),
    object_contour_color_(0, 0, 255),
    robot_contour_color_(255, 0, 0),
    change_color_(255, 0, 255),
    x_axis_color_(255, 0, 0),
    y_axis_color_(0, 0, 0),
    boundary_color_(0, 255, 255),
    waypoint_color_(255, 0, 255),
    window_name_(window_name), drawing_boundary_(false),
    // Tracking and state parameters
    tracking_(false), initialized_(false), paused_(false),
    min_contour_size_(0), run_count_(0), next_id_(0),
    // Robot Orientation Stuff
    initializing_orientation_(false), orientation_offset_(0.0),
    finished_orientation_init_(false), swap_orientation_(false),
    changed_waypoints_(false)
{
  boundary_contours_.clear();
  working_boundary_.clear();
  waypoints_.clear();
  reused_ids_.clear();

  // Create a HighGUI window for displaying and controlling the tracker
  namedWindow(window_name_);
  createTrackbar("Min Size", window_name_, &min_contour_size_,
                 MAX_MIN_SIZE);
  cvSetMouseCallback(window_name_.c_str(), onWindowClick, this);
  tracked_robot_.state.pose.x = 0;
  tracked_robot_.state.pose.y = 0;
  tracked_robot_.state.change.x = 0;
  tracked_robot_.state.change.y = 0;
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
    obj.id = getId();
    obj.state.pose.x = object_moments[i].m10 / object_moments[i].m00;
    obj.state.pose.y = object_moments[i].m01 / object_moments[i].m00;
    obj.state.change.x = 0;
    obj.state.change.y = 0;
    obj.moments = object_moments[i];
    tracked_objects_.push_back(obj);
  }
}

void OverheadTracker::initRobotTrack(vector<Point>& robot_contour,
                                     Moments& robot_moments)
{
  tracked_robot_.state.pose.x = robot_moments.m10 / robot_moments.m00;
  tracked_robot_.state.pose.y = robot_moments.m01 / robot_moments.m00;

  tracked_robot_.state.change.x = 0;
  tracked_robot_.state.change.y = 0;
  tracked_robot_.contour = robot_contour;
}


/**
 * Method to control setting the robot orientation offset.  On first call it
 * stores the current robot (x,y)-center.  On second call it finds the current
 * robot (x,y)-center and calculates the angle between the vector between the
 * centers and the horizontal axis.  The current robot orientation estimate is
 * subtracted from this and the orientation correction offset is stored.
 *
 * Subsequent calls iterate between these actions.
 */
void OverheadTracker::initializeOrientation()
{
  if(initializing_orientation_)
  {
    // Save current center
    double x_diff = tracked_robot_.state.pose.x - init_orientation_center_.x;
    double y_diff = tracked_robot_.state.pose.y - init_orientation_center_.y;

    // Calculate offset
    double theta_hat = atan2(y_diff, x_diff);
    orientation_offset_ = theta_hat - tracked_robot_.state.pose.theta;

    ROS_INFO("Set orientation offset to: %g", orientation_offset_);
    initializing_orientation_ = false;
    finished_orientation_init_ = true;
  }
  else
  {
    // Save current center
    init_orientation_center_.x = tracked_robot_.state.pose.x;
    init_orientation_center_.y = tracked_robot_.state.pose.y;

    ROS_INFO("Saved initial orientation center");
    orientation_offset_ = 0.0;
    initializing_orientation_ = true;
  }
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
  object_moments.clear();

  // Update the object tracks before drawing the updates on the display
  if (tracking_ && !paused_)
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
    ROS_DEBUG("Removed small contours");

    // Initialize tracking if it is currently not initialized
    if (!initialized_)
    {
      ROS_DEBUG("Initializing tracking on run %lu", run_count_);
      if (max_idx != -1)
      {
        initRobotTrack(contours[max_idx], object_moments[max_idx]);
        object_moments.erase(object_moments.begin() + max_idx);
        contours.erase(contours.begin() + max_idx);
      }
      initTracks(contours, object_moments);
    }
    else if(contours.size() > 0)
    {
      ROS_DEBUG("Updating tracks on run %lu", run_count_);
      if (max_idx != -1)
      {
        updateRobotTrack(contours[max_idx], object_moments[max_idx]);
        object_moments.erase(object_moments.begin() + max_idx);
        ROS_DEBUG("Erased robot moments");
        contours.erase(contours.begin() + max_idx);
        ROS_DEBUG("Erased robot contours");
      }
      ROS_DEBUG("Updating object tracks");
      updateTracks(contours, object_moments);
      ROS_DEBUG("Updated object tracks");
    }

    // Draw contours
    if (contours.size() > 0)
    {
      ROS_DEBUG("Drawing contours");
      drawContours(update_img, contours, -1, object_contour_color_, 2);
    }

    ROS_DEBUG("Drawing contour centers");
    // Draw contour centers
    for (unsigned int i = 0; i < tracked_objects_.size(); ++i)
    {
      Point center(tracked_objects_[i].state.pose.x,
                   tracked_objects_[i].state.pose.y);
      circle(update_img, center, 4, object_center_color_, 2);
      Point moved_center(tracked_objects_[i].state.pose.x -
                         tracked_objects_[i].state.change.x,
                         tracked_objects_[i].state.pose.y -
                         tracked_objects_[i].state.change.y);
      line(update_img, center, moved_center, change_color_, 2);

#ifdef SHOW_OBJ_IDS
      Point text_base(center.x + 4, center.y+4);
      stringstream num;
      num << tracked_objects_[i].id;
      putText(update_img, num.str(), text_base, FONT_HERSHEY_SIMPLEX, 1,
              change_color_, 3);
#endif
    }

    if (tracked_robot_.state.pose.x != 0 || tracked_robot_.state.pose.y != 0)
    {
      drawRobot(update_img);
    }
#ifdef SAVE_IMAGES
    if(fabs(orientation_offset_) > 0.0)
    {
      std::stringstream filename;
      filename << "/home/thermans/logs/tracking/track-" << (int) run_count_
               << ".png";
      imwrite(filename.str(), update_img);
    }
#endif
  }

  // Draw user defined boundaries
  if (boundary_contours_.size() > 0)
  {
    ROS_DEBUG("Drawing user boundaries");
    drawContours(update_img, boundary_contours_, -1,
                 boundary_color_, 2);
  }

  // Draw user defined waypoints
  if (waypoints_.size() > 0)
  {
    ROS_DEBUG("Drawing waypoints boundaries");
    circle(update_img, waypoints_[0], 2, waypoint_color_, 4);
    circle(update_img, waypoints_[0], 16, waypoint_color_, 2);
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
  // Store nearest neighbors in a multimap incase multiple centers map to the
  // same previous center
  multimap<int, int> min_idx_map;
  // Minimum distances are indexed by current object index
  vector<double> min_dists;
  vector<int> min_idx_vect;
  double max_min_dist = 0;
  // Match new regions to previous ones
  ROS_DEBUG("Finding nearest objects");
  for (unsigned int i = 0; i < object_moments.size() &&
           tracked_objects_.size() > 0; ++i)
  {
    double min_space_dist = DBL_MAX;
    int min_space_idx = -1;
    ROS_DEBUG("Scores for contour %u", i);
    for (unsigned int j = 0; j < tracked_objects_.size(); ++j)
    {
      double dX = tracked_objects_[j].state.pose.x - (object_moments[i].m10 /
                                                      object_moments[i].m00);
      double dY = tracked_objects_[j].state.pose.y - (object_moments[i].m01 /
                                                      object_moments[i].m00);
      double space_dist = sqrt(dX*dX + dY*dY);
      if (space_dist < min_space_dist)
      {
        min_space_dist = space_dist;
        min_space_idx = j;
      }
    }
    // We have found the mimimum score, update our instances...
    ROS_DEBUG("Min spacial distnace is %le at point %u",
              min_space_dist, min_space_idx);
    if (min_space_dist > MIN_DIST_THRESH)
    {
      min_idx_vect.push_back(-1);
      min_dists.push_back(0.0);
      min_space_dist = 0.0;
    }
    else
    {
      min_idx_map.insert(pair<int,int>(min_space_idx, i));
      min_idx_vect.push_back(min_space_idx);
      min_dists.push_back(min_space_dist);
    }
    if (min_space_dist > max_min_dist)
    {
      max_min_dist = min_space_dist;
    }

  }

  ROS_DEBUG("Sorting multiple objects");
  // Sort out multiple objects mapped to the same previous object
  for(unsigned int i = 0; i < tracked_objects_.size(); ++i)
  {
    if(min_idx_map.count(i) > 1)
    {
      // Find the minimum dist of all of the possible objects
      multimap<int,int>::iterator it;
      int min_k = -1;
      vector<int> ks;
      double min_space_dist = DBL_MAX;
      for(it = min_idx_map.equal_range(i).first;
          it != min_idx_map.equal_range(i).second; ++it)
      {
        int k = it->second;
        ks.push_back(k);
        if (min_dists[k] < min_space_dist)
        {
          min_k = k;
          min_space_dist = min_dists[k];
        }
      }

      // Those that aren't minimum dist, set to be new objects
      for(unsigned int j = 0; j < ks.size(); ++j)
      {
        int k = ks[j];
        if (k != min_k)
        {
          min_idx_vect[k] = -1;
        }
      }
    }
  }

  ROS_DEBUG("Updating objects");
  vector<OverheadVisualObject> new_objects;
  // Now that we've matched tracks update the current ones
  for (unsigned int i = 0; i < object_moments.size() && i < min_idx_vect.size();
       ++i)
  {
    ROS_DEBUG("Getting previous index for i = %d", i);
    int prev_idx = min_idx_vect[i];
    ROS_DEBUG("Previous index was: %d", prev_idx);
    OverheadVisualObject obj;
    obj.state.pose.x = object_moments[i].m10 / object_moments[i].m00;
    obj.state.pose.y = object_moments[i].m01 / object_moments[i].m00;
    obj.moments = object_moments[i];
    // This is a new object
    if (prev_idx < 0 || prev_idx > static_cast<int>(tracked_objects_.size()))
    {
      obj.state.change.x = 0.0;
      obj.state.change.y = 0.0;
      obj.id = getId();
      new_objects.push_back(obj);
    }
    else
    {
      obj.state.change.x = obj.state.pose.x -
          tracked_objects_[prev_idx].state.pose.x;
      obj.state.change.y = obj.state.pose.y -
          tracked_objects_[prev_idx].state.pose.y;
      obj.id = tracked_objects_[prev_idx].id;
      tracked_objects_[prev_idx] = obj;
    }
  }

  ROS_DEBUG("Remove unmatched objects");
  // Now that matching is done, remove unmatched objects from the previous frame
  unsigned int offset = 0;
  for (unsigned int i = 0; i < tracked_objects_.size();)
  {
    if (min_idx_map.count(i + offset) < 1)
    {
      releaseId(tracked_objects_[i].id);
      tracked_objects_.erase(tracked_objects_.begin() + i);
      ++offset;
    }
    else
    {
      ++i;
    }
  }

  ROS_DEBUG("Add new objects");
  // Now add new objects which were not matched to objects in the previos frame
  for (unsigned int i = 0; i < new_objects.size(); ++i)
  {
    tracked_objects_.push_back(new_objects[i]);
  }
}

/**
 * Update the state of the robot for the current frame
 *
 * @param robot_contour Bounding contour of the robot
 * @param robot_moments Moments from the contour.
 */
void OverheadTracker::updateRobotTrack(vector<Point>& robot_contour,
                                       Moments& robot_moments)
{
  double newX = robot_moments.m10 / robot_moments.m00;
  double newY = robot_moments.m01 / robot_moments.m00;
  tracked_robot_.state.change.x = newX - tracked_robot_.state.pose.x;
  tracked_robot_.state.change.y = newY - tracked_robot_.state.pose.y;

  tracked_robot_.state.pose.x = newX;
  tracked_robot_.state.pose.y = newY;

  tracked_robot_.contour = robot_contour;
  tracked_robot_.moments = robot_moments;
  if ((tracked_robot_.state.change.x*tracked_robot_.state.change.x +
       tracked_robot_.state.change.y*tracked_robot_.state.change.y) > 250000)
  {
    tracked_robot_.state.change.x = 0;
    tracked_robot_.state.change.y = 0;
  }

  // Get robot orientation
  double theta_prime = (0.5*atan2(2*robot_moments.mu11, robot_moments.mu20 -
                                  robot_moments.mu02) +
                        orientation_offset_);

  // Swap the orientation by pi radians
  if(swap_orientation_ && !initializing_orientation_ )
  {
    if(theta_prime > 0.0)
      theta_prime = theta_prime - M_PI;
    else
      theta_prime = theta_prime + M_PI;
  }

  if (finished_orientation_init_)
  {
    // Don't worry about swapping if we just finished initializing
    finished_orientation_init_ = false;
    if (swap_orientation_)
    {
      if(theta_prime > 0.0)
        theta_prime = theta_prime - M_PI;
      else
        theta_prime = theta_prime + M_PI;
    }
    swap_orientation_ = false;
  }
  else if( !initializing_orientation_)
  {
    // If the orientation has switched signs since last frame and the theta is
    // not near zero, then we need to switch whether we swap the orientation by
    // pi radians
    if ((sign(tracked_robot_.state.pose.theta) != sign(theta_prime))
        &&
        ((fabs(tracked_robot_.state.pose.theta) > 1.0 &&
          fabs(tracked_robot_.state.pose.theta) < (M_PI - 0.5))
          ||
          (fabs(theta_prime) > 1.0 && fabs(theta_prime) < (M_PI - 0.5))
         ))
    {
      swap_orientation_ = !swap_orientation_;
      ROS_DEBUG("Swapping orientation direction to %d", swap_orientation_);
      ROS_DEBUG("Tracked robot theta is %f", tracked_robot_.state.pose.theta);
      ROS_DEBUG("Theta prime is %f", theta_prime);
      // We either need to swap or need to undo the swap
      if(theta_prime > 0.0)
        theta_prime = theta_prime - M_PI;
      else
        theta_prime = theta_prime + M_PI;
    }
  }

  ROS_DEBUG("Updating robot state.");
  tracked_robot_.state.pose.theta = theta_prime;
  ROS_DEBUG("Updated robot state.");
}

//
// User IO Methods
//

/**
 * Draw the robot contour, center, and axis on the image
 *
 * @param draw_on Image to be drawn on
 */
void OverheadTracker::drawRobot(Mat& draw_on)
{
  vector<vector<Point> > robot_contours;
  robot_contours.push_back(tracked_robot_.contour);
  Point center(tracked_robot_.state.pose.x,
               tracked_robot_.state.pose.y);
  Point moved_center(tracked_robot_.state.pose.x -
                     tracked_robot_.state.change.x,
                     tracked_robot_.state.pose.y -
                     tracked_robot_.state.change.y);
  // Calculate robot axes points
  Point x_a(center.x + cos(tracked_robot_.state.pose.theta)*100,
            center.y + sin(tracked_robot_.state.pose.theta)*100);
  Point y_a(center.x + cos(tracked_robot_.state.pose.theta - M_PI/2.0)*100,
            center.y + sin(tracked_robot_.state.pose.theta - M_PI/2.0)*100);

  // Draw boundary
  drawContours(draw_on, robot_contours, -1, robot_contour_color_, 2);
  // Draw axes
  line(draw_on, center, x_a, x_axis_color_, 2);
  line(draw_on, center, y_a, y_axis_color_, 2);
  // Draw Center
  circle(draw_on, center, 4, robot_contour_color_, 2);
  // Draw dX vector
  line(draw_on, center, moved_center, change_color_, 2);
}

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
    ROS_INFO("Adding boundary point.");
  }
}

/**
 * Method to draw a waypoint for the robot and publish it over ROS
 *
 * @param pt The on screen point clicked by the user
 */
void OverheadTracker::setRobotWaypoint(Point pt)
{
  waypoints_.push_back(pt);
  changed_waypoints_ = true;
  // Convert to global coordinates

  // Publish over ros
}

/**
 * Perform the requist action dependent on the user input key
 *
 * @param c The input key
 */
void OverheadTracker::onKeyCallback(char c)
{
  // Perform actions corresponding to key presses
  switch(c)
  {
    case TOGGLE_TRACKING_KEY:
      if (!tracking_ && bg_->hasBackgroundImage())
      {
        tracking_ = true;
        ROS_INFO("Begining tracking.");
      }
      else if(tracking_)
      {
        tracking_ = false;
        initialized_ = false;
        paused_ = false;
        tracked_objects_.clear();
        ROS_INFO("Tracking ended.");
      }
      else
      {
        ROS_ERROR("Tracking key hit, but not doing anything!");
      }
      break;
    case PAUSE_TRACKING_KEY:
      if(tracking_)
      {
        paused_ = !paused_;
        if(paused_)
          ROS_INFO("Tracking paused.");
        else
          ROS_INFO("Tracking unpaused.");
      }
      break;
    case INIT_ORIENTATION_KEY:
      if(tracking_)
        initializeOrientation();
      break;
    case CLEAR_WAYPOINT_KEY:
      waypoints_.clear();
      break;
    case DRAW_BOUNDARY_KEY:
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
      break;
    case CLEAR_BOUNDARIES_KEY:
      boundary_contours_.clear();
      ROS_INFO("Cleared boundaries.");
      break;
    case CLEAR_WORKING_BOUNDARY_KEY:
      working_boundary_.clear();
      ROS_INFO("Cleared working boundary.");
      break;
    default:
      break;
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
      if (!tracker->addingContour())
      {
        tracker->setRobotWaypoint(pt);
      }
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

/**
 * Get the next free ID for the tracked objects.
 *
 * @return The next current id
 */
int OverheadTracker::getId()
{
  if (reused_ids_.size() < 1)
    return next_id_++;
  int newId = reused_ids_[reused_ids_.size() - 1];
  reused_ids_.pop_back();
  return newId;
}

/**
 * Release the ID used by a now untracked object
 *
 * @param i The ID to release
 */
void OverheadTracker::releaseId(int i)
{
  reused_ids_.push_back(i);
}

//
// Methods to convert image coordinates to world coordinates
// Instead of doing a calibration, this is just some cheap crap right now...
//

/**
 * Translate x in image frame to x in the world frame
 *
 * @param img_x The x location in the image
 *
 * @return The estimate of x in the world frame
 */
double OverheadTracker::imgXtoWorldX(double img_x)
{
  return img_x*WORLD_MAX_X/CAMERA_MAX_X;
}

/**
 * Translate y in image frame to y in the world frame
 *
 * @param img_y The y location in the image
 *
 * @return The estimate of y in the world frame
 */
double OverheadTracker::imgYtoWorldY(double img_y)
{
  return img_y*WORLD_MAX_Y/CAMERA_MAX_Y;
}

//
// Getters for tracked objects in world frame
//

/**
 * Get the vector of cleanup objects in the world frame
 *
 * @return the array of cleanup objects
 */
CleanupObjectArray OverheadTracker::getCleanupObjects()
{
  CleanupObjectArray cleanup_objs;
  for(unsigned int i = 0; i < tracked_objects_.size(); ++i)
  {
    CleanupObject obj;
    obj.pose.x = imgXtoWorldX(tracked_objects_[i].state.pose.x);
    obj.pose.y = imgYtoWorldY(tracked_objects_[i].state.pose.y);
    obj.change.x = imgXtoWorldX(tracked_objects_[i].state.change.x);
    obj.change.y = imgYtoWorldY(tracked_objects_[i].state.change.y);

    cleanup_objs.objects.push_back(obj);
  }

  return cleanup_objs;
}

/**
 * Get the pose of the robot in the world frame
 *
 * @return the pose of the robot in the world frame
 */
Pose2D OverheadTracker::getRobotPose()
{
  Pose2D world_pose;
  world_pose.x = imgXtoWorldX(tracked_robot_.state.pose.x);
  world_pose.y = imgYtoWorldY(tracked_robot_.state.pose.y);
  world_pose.theta = tracked_robot_.state.pose.theta;

  return world_pose;
}

Pose2D OverheadTracker::getGoalPose()
{
  Pose2D goal_pose;
  if (waypoints_.size() > 0)
  {
    goal_pose.x = waypoints_[0].x;
    goal_pose.y = waypoints_[0].y;
    goal_pose.theta = tracked_robot_.state.pose.theta;
  }
  else
  {
    goal_pose.x = -1337;
    goal_pose.y = -1337;
    goal_pose.theta = -1337;
  }
  return goal_pose;
}

bool OverheadTracker::newGoalPose()
{
  if(changed_waypoints_)
  {
    changed_waypoints_ = false;
    return true;
  }
  return false;
}
