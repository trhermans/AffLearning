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

#ifndef bg_subtract_h_DEFINED
#define bg_subtract_h_DEFINED

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

/**
 * @file   bg_subtract.h
 * @author Tucker Hermans <thermans@cc.gatech.edu>
 * @date   Mon Feb 15 10:10:36 2010
 *
 * @brief Class to perform background subtraction
 *
 */
class BgSubtract
{
 public:
  BgSubtract(cv::Mat bg_mg);
  BgSubtract();
  cv::Mat subtract(cv::Mat fg_img, int thresh = 0);
  cv::Mat findFGContours(cv::Mat fg_img, int thresh = 0, int min_size = 0);
  void updateBgImage(const cv::Mat bg_img);
  void removeBgImage();

  // Getters and Setters
  bool hasBackgroundImage() const
  {
    return has_bg_img_;
  }
  const std::vector<std::vector<cv::Point> > getContours() const
  {
    return contours_;
  }

 protected:
  bool has_bg_img_;
  cv::Mat bg_img_;
  std::vector<std::vector<cv::Point> > contours_;
};

class BgSubtractGUI
{
 public:
  BgSubtractGUI();
  BgSubtractGUI(cv::Mat bg_img);
  ~BgSubtractGUI() {}
  void updateDisplay(cv::Mat update_img);
  BgSubtract bg_sub_;

 protected:
  int diff_thresh_;
  bool active_display_;

  // Constants
  static const int MAX_DIFF_THRESH;
  static const char CREATE_BG_KEY;
  static const char ERASE_BG_KEY;
  static const char ACTIVE_DISPLAY_KEY;

 private:
  void raiseDisplay();
};

#endif // bg_subtract_h_DEFINED
