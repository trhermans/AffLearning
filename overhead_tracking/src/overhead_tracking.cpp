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

// Constants
const int OverheadTracker::MAX_MIN_SIZE = 1000;
const unsigned int OverheadTracker::MIN_NUM_CONTOUR_POINTS = 3;
const char OverheadTracker::DRAW_BOUNDARY_KEY = 'd';
const char OverheadTracker::CLEAR_BOUNDARIES_KEY = 'k';
const char OverheadTracker::CLEAR_WORKING_BOUNDARY_KEY = 'w';

OverheadTracker::OverheadTracker(String window_name) :
        object_center_color_(0, 255, 0),
        object_contour_color_(0, 0, 255),
        boundary_color_(0, 255, 255),
        window_name_(window_name), drawing_boundary_(false),
        min_contour_size_(0)
{
    boundary_contours_.clear();
    working_boundary_.clear();

    // Create a HighGUI window for displaying and controlling the tracker
    namedWindow(window_name_);
    createTrackbar("Min Size", window_name_, &min_contour_size_,
                   MAX_MIN_SIZE);
    cvSetMouseCallback(window_name_.c_str(), onWindowClick, this);
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
    // Clear out the contour momoments from the previous frame
    contour_moments_.clear();

    // Copy the image so that we don't get lots of old object boundaries drawn
    // when the update image is slow to refresh
    Mat update_img;
    update_img_raw.copyTo(update_img);

    // Iterate through the contours, removing those with area less than min_size
    for (unsigned int i = 0; i < contours.size();)
    {
        double size = contourArea(contours[i]);
        if (fabs(size) > min_contour_size_)
        {
            contour_moments_.push_back(moments(contours[i]));
            ++i;
        }
        else
        {
            contours.erase(contours.begin()+i);
        }
    }

    // Draw contour centers
    for (unsigned int i = 0; i < contour_moments_.size(); ++i)
    {
        Point center(contour_moments_[i].m10 / contour_moments_[i].m00,
                     contour_moments_[i].m01 / contour_moments_[i].m00);
        circle(update_img, center, 4, object_center_color_, 2);
    }

    if (contours.size() > 0)
    {
        drawContours(update_img, contours, -1, object_contour_color_, 2);
    }

    if (boundary_contours_.size() > 0)
    {
        drawContours(update_img, boundary_contours_, -1,
                     boundary_color_, 2);
    }

    // Now show our image
    imshow(window_name_, update_img);
    char c = waitKey(3);

    // Perform actions corresponding to key presses
    if (c == DRAW_BOUNDARY_KEY)
    {
        if (drawing_boundary_)
        {
            // Accept the current boundary as a contour
            if (working_boundary_.size() >= MIN_NUM_CONTOUR_POINTS) {
                boundary_contours_.push_back(working_boundary_);
            }
            working_boundary_.clear();
        }

        // Toggle drawing boundary on and off
        drawing_boundary_ = ! drawing_boundary_;
    }

    if (c == CLEAR_BOUNDARIES_KEY)
    {
        boundary_contours_.clear();
    }

    if (c == CLEAR_WORKING_BOUNDARY_KEY)
    {
        working_boundary_.clear();
    }
}

/**
 * Add a point to the currently building boundary contour
 *
 * @param pt The point to be added
 */
void OverheadTracker::addBoundaryPoint(cv::Point pt)
{
    if (drawing_boundary_)
    {
        working_boundary_.push_back(pt);
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
