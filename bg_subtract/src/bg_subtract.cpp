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

#include "BgSubtract.hpp"
#include <vector>

using namespace cv;
using namespace std;

/**
 * Initialize the class with a background image
 *
 * @param _bgImg Background image for the class
 */
BgSubtract::BgSubtract(Mat& bg_img): bg_img_(bg_img)
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
Mat BgSubtract::subtract(Mat& fg_img, uchar thresh)
{
    // Determine which pixels are different
    Mat diff_img = abs(fg_img - bg_img_);

    // Apply this mask to the foreground image
    vector<Mat> diff_planes;
    vector<Mat> fg_planes;
    split(diff_img, diff_planes);
    split(fg_img, fg_planes);

    for(int y = 0; y < diff.rows; ++y)
    {
        for(int x = 0; x < diff.cols; ++x)
        {
            if(diff_planes[0].at<uchar>(y,x) > thresh ||
               diff_planes[1].at<uchar>(y,x) > thresh ||
               diff_planes[2].at<uchar>(y,x) > thresh)
            {
                diff_planes[0].at<uchar>(y,x) = fg_planes[0].at<uchar>(y,x);
                diff_planes[1].at<uchar>(y,x) = fg_planes[1].at<uchar>(y,x);
                diff_planes[2].at<uchar>(y,x) = fg_planes[2].at<uchar>(y,x);
            }
        }
    }

    merge(diff_planes, diff_img);
    return diff_img;
}

Mat BgSubtract::getContours(Mat& fg_img, uchar thresh)
{
    Mat diff_img = subtract(fg_img, thresh);
    Mat bw_diff(diff_img.size(), CV_8UC1);
    cvtColor(diff_img, bw_diff, CV_RGB2GRAY);

    vector<vector<Point> > contours;

    findContours(bw_diff, contours, RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    Scalar color(0, 255, 0);
    drawContours(fg_img, contours, -1, color, 2);

    return fg_img;
}

void BgSubtract::updateBgImage(cv::Mat& bg_img)
{
    bg_img_ = bg_img;
}
