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
const int OverheadTracker::MAX_MIN_SIZE = 500;

OverheadTracker::OverheadTracker(String window_name) :
        min_contour_size_(0), center_color_(0,255,0), contour_color_(0,0,255),
        window_name_(window_name)
{
    namedWindow(window_name_);
    createTrackbar("Min Size", window_name_, &min_contour_size_,
                   MAX_MIN_SIZE);
}

void OverheadTracker::updateDisplay(Mat update_img,
                                    vector<vector<Point> > contours)
{
    // Clear out the contour momoments from the previous frame
    contour_moments_.clear();

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
        circle(update_img, center, 4, center_color_, 2);
    }

    if (contours.size() > 0)
    {
        drawContours(update_img, contours, -1, contour_color_, 2);
    }

    // Now show our image
    imshow(window_name_, update_img);
}
