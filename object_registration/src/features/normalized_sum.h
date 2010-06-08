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

#ifndef normalized_sum_h_DEFINED
#define normalized_sum_h_DEFINED

#include <opencv2/core/core.hpp>

class NormalizedSum
{
 public:
  NormalizedSum() :
      max_sum(-1.0), max_loc(0, 0, 0, 0)
  {
  }

  float operator()(cv::Mat& img, cv::Rect& window)
  {
    float area_sum = 0.0;
    for (int i = 0; i < img.rows; ++i)
    {
      for (int j = 0; j < img.cols; ++j)
      {
        area_sum += img.at<uchar>(i,j);
      }
    }

    area_sum /= (img.cols*img.rows);

    if (area_sum > max_sum)
    {
      max_sum = area_sum;
      max_loc.x = window.x;
      max_loc.y = window.y;
      max_loc.height = window.height;
      max_loc.width = window.width;
    }

    return area_sum;
  }

  float getMax() const { return max_sum; }
  cv::Rect getMaxLoc() const { return max_loc; }
  void resetMax()
  {
    max_sum = -1.0;
    max_loc.x = 0.0;
    max_loc.y = 0.0;
    max_loc.height = 0.0;
    max_loc.width = 0.0;
  }
 protected:
  float max_sum;
  cv::Rect max_loc;
};
#endif // normalized_sum_h_DEFINED
