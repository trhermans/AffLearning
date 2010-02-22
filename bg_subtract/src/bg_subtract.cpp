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
#include "bg_subtract.h"
#include <opencv/highgui.h>
#include <iostream>

using namespace cv;
using std::vector;

/*
 * BgSubtract Class
 */

/**
 * Initialize the class with a background image
 *
 * @param _bgImg Background image for the class
 */
BgSubtract::BgSubtract(Mat bg_img) :
        has_bg_img_(true), bg_img_(bg_img), contour_color_(0, 0, 255)
{
}

BgSubtract::BgSubtract() :
        has_bg_img_(false), contour_color_(0, 0, 255)
{
}

/**
 * Function performs background subtraction on a given image of items atop the
 * stored background image
 *
 * @param fgImg The new image
 * @param thresh value for what is considered no change. Default is 0.
 *
 * @return the foreground image without the background
 */
Mat BgSubtract::subtract(Mat fg_img, int thresh)
{
    // Determine which pixels are different
    Mat diff_img = abs(fg_img - bg_img_);

    // Apply this mask to the foreground image
    vector<Mat> diff_planes;
    vector<Mat> fg_planes;
    split(diff_img, diff_planes);
    split(fg_img, fg_planes);

    for(int y = 0; y < diff_img.rows; ++y)
    {
        for(int x = 0; x < diff_img.cols; ++x)
        {
            if(diff_planes[0].at<uchar>(y,x) > (uchar) thresh ||
               diff_planes[1].at<uchar>(y,x) > (uchar) thresh ||
               diff_planes[2].at<uchar>(y,x) > (uchar) thresh)
            {
                diff_planes[0].at<uchar>(y,x) = fg_planes[0].at<uchar>(y,x);
                diff_planes[1].at<uchar>(y,x) = fg_planes[1].at<uchar>(y,x);
                diff_planes[2].at<uchar>(y,x) = fg_planes[2].at<uchar>(y,x);
            } else {
                diff_planes[0].at<uchar>(y,x) = (uchar) 0;
                diff_planes[1].at<uchar>(y,x) = (uchar) 0;
                diff_planes[2].at<uchar>(y,x) = (uchar) 0;
            }
        }
    }

    merge(diff_planes, diff_img);
    return diff_img;
}

/**
 * Perform background subtraction and return the most recent image with the
 * contours of the foreground objects outlined in green!
 *
 * @param fg_img Current image
 * @param thresh bg subtract threshold value
 *
 * @return Input image with contours drawn on it
 */
Mat BgSubtract::findFGContours(Mat fg_img, int thresh, int min_size)
{
    Mat diff_img = subtract(fg_img, thresh);
    Mat bw_diff(diff_img.size(), CV_8UC1);
    cvtColor(diff_img, bw_diff, CV_RGB2GRAY);

    contours_.clear();
    contour_moments_.clear();

    findContours(bw_diff, contours_, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // Iterate through the contours, removing those with area less than min_size
    for (unsigned int i = 0; i < contours_.size();)
    {
        double size = contourArea(contours_[i]);
        if (fabs(size) > min_size)
        {
            contour_moments_.push_back(moments(contours_[i]));
            ++i;
        }
        else
        {
            contours_.erase(contours_.begin()+i);
        }
    }

    drawContours(fg_img, contours_, -1, contour_color_, 2);

    // Draw contour centers
    for (unsigned int i = 0; i < contour_moments_.size(); ++i)
    {
        Point center(contour_moments_[i].m10 / contour_moments_[i].m00,
                     contour_moments_[i].m01 / contour_moments_[i].m00);
        Scalar s(0,255,0);
        circle(fg_img, center, 4, s, 2);
    }

    return fg_img;
}

/**
 * Update the background image for bg subtraction.
 *
 * @param bg_img The new background image.
 */
void BgSubtract::updateBgImage(const Mat bg_img)
{
    bg_img.copyTo(bg_img_);
    has_bg_img_ = true;
}

void BgSubtract::removeBgImage()
{
    has_bg_img_ = false;
}

/*
 * BgSubtractGUI Class
 */

const int BgSubtractGUI::MAX_DIFF_THRESH = 255;
const int BgSubtractGUI::MAX_MIN_SIZE = 500;
const char BgSubtractGUI::CREATE_BG_KEY = 'c';
const char BgSubtractGUI::ERASE_BG_KEY = 'e';
const char BgSubtractGUI::PRINT_CONTOUR_KEY = 'p';

BgSubtractGUI::BgSubtractGUI() :
        diff_thresh_(0), min_contour_size_(0)
{
    raiseDisplay();
}

BgSubtractGUI::BgSubtractGUI(Mat bg_img) :
        bg_sub_(bg_img), diff_thresh_(0), min_contour_size_(0)
{
    raiseDisplay();
}

void BgSubtractGUI:: raiseDisplay()
{
    namedWindow("Background Subtract");
    createTrackbar("Threshold", "Background Subtract", &diff_thresh_,
                   MAX_DIFF_THRESH);
    namedWindow("Contours");
    createTrackbar("Min Size", "Contours", &min_contour_size_,
                   MAX_MIN_SIZE);

}

void BgSubtractGUI::updateDisplay(Mat update_img)
{
    if( bg_sub_.hasBackgroundImg() )
    {
        Mat to_display = bg_sub_.subtract(update_img, diff_thresh_);
        Mat to_display_2 = bg_sub_.findFGContours(update_img, diff_thresh_,
                                                  min_contour_size_);
        imshow("Background Subtract", to_display);
        imshow("Contours", to_display_2);
    }
    else
    {
        imshow("Background Subtract", update_img);
    }
    char c = cvWaitKey(3);

    if (c == CREATE_BG_KEY)
    {
        bg_sub_.updateBgImage(update_img);
    }
    else if (c == ERASE_BG_KEY)
    {
        bg_sub_.removeBgImage();
    }

    if (c == PRINT_CONTOUR_KEY)
    {
        vector<Moments> c_ms = bg_sub_.getContourMoments();
        vector<vector<Point> > cts = bg_sub_.getContours();

        for (unsigned int i = 0; i < c_ms.size(); ++i)
        {
            double size = contourArea(cts[i]);
            ROS_INFO_STREAM( "Contour " << i << " has area " << -size << " px^2"
                             << " at ("
                             << (int) (c_ms[i].m10 / c_ms[i].m00) << ", "
                             << (int) (c_ms[i].m01 / c_ms[i].m00) << ")");
        }
    }
}
