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

#include "sliding_window.h"
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <iostream>

#include "saliency/center_surround.h"
#include "features/normalized_sum.h"

using cv::Mat;
using cv::Rect;
using std::pair;
using std::vector;

int main(int argc, char** argv)
{
  int count = 1;

  if (argc > 1)
    count = atoi(argv[1]);

  vector<pair<int,int> > windows;
  windows.push_back(pair<int,int>(  8,   8));
  windows.push_back(pair<int,int>( 16,   8));
  windows.push_back(pair<int,int>(  8,  16));
  windows.push_back(pair<int,int>( 32,   8));
  windows.push_back(pair<int,int>(  8,  32));
  windows.push_back(pair<int,int>( 16,  16));
  windows.push_back(pair<int,int>( 16,  32));
  windows.push_back(pair<int,int>( 32,  16));
  windows.push_back(pair<int,int>( 32,  32));
  windows.push_back(pair<int,int>( 64,  32));
  windows.push_back(pair<int,int>( 32,  64));
  windows.push_back(pair<int,int>( 64,  64));
  windows.push_back(pair<int,int>( 32, 128));
  windows.push_back(pair<int,int>(128,  32));
  windows.push_back(pair<int,int>( 64, 128));
  windows.push_back(pair<int,int>(128,  64));
  windows.push_back(pair<int,int>( 64, 256));
  windows.push_back(pair<int,int>(256,  64));
  windows.push_back(pair<int,int>(128, 128));
  windows.push_back(pair<int,int>(128, 256));
  windows.push_back(pair<int,int>(256, 128));
  windows.push_back(pair<int,int>(256, 256));
  windows.push_back(pair<int,int>(128, 512));
  windows.push_back(pair<int,int>(512, 128));
  windows.push_back(pair<int,int>(256, 512));
  windows.push_back(pair<int,int>(512, 256));

  SlidingWindowDetector<NormalizedSum> swd;
  CenterSurroundMapper csm(1, 3);

  for (int i = 0; i < count; i++)
  {
    std::stringstream filepath;
    filepath << "/home/thermans/data/robot-frames/test1/" << i << ".png";
    //filepath << "/home/thermans/data/robot.jpg";
    std::cout << "Image " << i << std::endl;
    Mat frame;
    frame = cv::imread(filepath.str());
    Mat bw_frame(frame.rows, frame.cols, CV_8UC1);
    cvtColor(frame, bw_frame, CV_RGB2GRAY);

    try
    {

      // Get the saliency map
      Mat saliency_img = csm(frame);
      cv::imshow("saliency map scaled", saliency_img);
      cv::waitKey(6);

      // Find the most salient region
      swd.scanImage(saliency_img, windows);

      // Report what this region is
      cv::Rect max_loc = swd.feature_.getMaxLoc();

      std::cout << "max_loc: ("
                << max_loc.x << ", " << max_loc.y << ", "
                << max_loc.height << ", " << max_loc.width
                << ")" << std::endl;

      // Display stuff
      // Mat disp_img(saliency_img.rows, saliency_img.cols, frame.type());
      // cv::resize(frame, disp_img, disp_img.size());
      Mat disp_img(frame.rows, frame.cols, frame.type());
      frame.copyTo(disp_img);
      int img_scale = frame.cols / saliency_img.cols;
      cv::rectangle(disp_img, max_loc.tl()*img_scale, max_loc.br()*img_scale,
                    CV_RGB(255,0,0));

      cv::imshow("Most salient region", disp_img);
      cv::waitKey(30);
    }
    catch(cv::Exception e)
    {
      std::cerr << e.err << std::endl;
    }
  }
  return 0;
}
