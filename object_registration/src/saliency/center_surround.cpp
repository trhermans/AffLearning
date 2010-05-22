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

#include "center_surround.h"
#include <opencv/highgui.h>

#include <iostream>
#include <sstream>


using cv::Mat;
using std::vector;

CenterSurroundMapper::CenterSurroundMapper() :
    num_scales_(8)
{
  scales_.clear();
}

Mat CenterSurroundMapper::operator()(Mat& frame)
{
  Mat i_frame(frame.rows, frame.cols, CV_8UC1);

  cvtColor(frame, i_frame, CV_RGB2GRAY);

  Mat R(frame.rows, frame.cols, CV_8UC1);
  Mat G(frame.rows, frame.cols, CV_8UC1);
  Mat B(frame.rows, frame.cols, CV_8UC1);
  Mat Y(frame.rows, frame.cols, CV_8UC1);

  vector<Mat> channels;

  split(frame, channels);
  cv::imshow("red", channels[0]);
  cv::imshow("green", channels[1]);
  cv::imshow("blue", channels[2]);
  cv::waitKey();

  cv::buildPyramid(i_frame, I_scales_, num_scales_);
  cv::buildPyramid(r_frame, R_scales_, num_scales_);
  cv::buildPyramid(g_frame, G_scales_, num_scales_);
  cv::buildPyramid(b_frame, B_scales_, num_scales_);
  cv::buildPyramid(y_frame, Y_scales_, num_scales_);

  Mat saliency_map(frame.rows, frame.cols, CV_8UC1);
  cvtColor(frame, saliency_map, CV_RGB2GRAY);

  R = channels[0] - (channels[1] + channels[2])/2.0;
  G = channels[1] - (channels[0] + channels[2])/2.0;
  B = channels[2] - (channels[0] + channels[1])/2.0;
  Y = ((channels[0] + channels[1])/2.0 - (channels[0] - channels[1])/2.0
       - channels[2]);

  return saliency_map;
}

Mat CenterSurroundMapper::mapDifference(Mat& s, Mat& c)
{
  // Upsample c to s resolution
  Mat s_prime(c.rows, c.cols. CV_8UC1);
  cv::pyrUp(s, s_prime, cv::Size(s_prime.cols, s_prime.rows));

  // Take pixelwise difference
  Mat diff = c - s_prime;
  // Return this map
  return diff;
}
