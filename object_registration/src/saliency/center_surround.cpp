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

  Mat r_frame(frame.rows, frame.cols, CV_8UC1);
  Mat g_frame(frame.rows, frame.cols, CV_8UC1);
  Mat b_frame(frame.rows, frame.cols, CV_8UC1);
  Mat y_frame(frame.rows, frame.cols, CV_8UC1);

  r_frame = frame

  cv::buildPyramid(i_frame, I_scales_, num_scales_);
  cv::buildPyramid(r_frame, R_scales_, num_scales_);
  cv::buildPyramid(g_frame, G_scales_, num_scales_);
  cv::buildPyramid(b_frame, B_scales_, num_scales_);
  cv::buildPyramid(y_frame, Y_scales_, num_scales_);

  Mat saliency_map(frame.rows, frame.cols, CV_8UC1);
  cvtColor(frame, saliency_map, CV_RGB2GRAY);
  return saliency_map;
}
