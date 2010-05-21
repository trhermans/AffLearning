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

#ifndef sliding_window_h_DEFINED
#define sliding_window_h_DEFINED

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <vector>
#include <utility>

template<class FeatureType> class SlidingWindowDetector
{
 public:
  /**
   * Default constructor
   */
  SlidingWindowDetector() : feature_()
  {
    windows_.clear();
  }

  /**
   * Function wrapper to perform the sliding window task across a given image for
   * an arbitrary number of window shapes and sizes, using the class set windows
   *
   * @param img The image to perform sliding windows over
   */
  void scanImage(cv::Mat& img)
  {
    for (unsigned int i = 0; i < windows_.size(); ++i)
    {
      scanImage(img, windows_[i].first, windows_[i].second);
    }
  }

  /**
   * Function wrapper to perform the sliding window task across a given image for
   * an arbitrary number of window shapes and sizes
   *
   * @param img The image to perform sliding windows over
   * @param windows A vector of integer pairs of window height and widths
   */
  void scanImage(cv::Mat& img, std::vector<std::pair<int, int> >& windows)
  {
    for (unsigned int i = 0; i < windows.size(); ++i)
    {
      scanImage(img, windows[i].first, windows[i].second);
    }
  }

  /**
   * Workhorse function of the class that extracts the specified windows across
   * the image and sends them to the callback task
   *
   * @param img The image to perform sliding windows over
   * @param window_width Width of the sliding window to be used
   * @param window_height Height of the sliding window to be used
   */
  void scanImage(cv::Mat& img, int window_width, int window_height)
  {
    // Scan the window horizontally first
    for (int r = 0; r + window_height < img.rows; r++)
    {
      for (int c = 0; c + window_width < img.cols; c++)
      {
        cv::Rect roi(c, r, window_width, window_height);
        cv::Mat window = img(roi);
        feature_(window);
      }
    }
  }

  void setWindowSizes(std::vector<std::pair<int, int> > windows)
  {
    windows_ = windows;
  }

 protected:
  std::vector<std::pair<int, int> > windows_;

 public:
  FeatureType feature_;
};

#endif // sliding_window_h_DEFINED
