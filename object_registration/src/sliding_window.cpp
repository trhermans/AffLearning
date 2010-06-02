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
#include <opencv/highgui.h>
#include <sstream>
#include <iostream>

#include "features/color_histogram.h"
#include "saliency/center_surround.h"

using cv::Mat;
using cv::Rect;
using std::pair;
using std::vector;

int main(int argc, char** argv)
{
  int count = 1;
  if (argc > 1)
    count = atoi(argv[1]);

  SlidingWindowDetector<ColorHistogram<12> > swd;
  CenterSurroundMapper csm;

  vector<pair<int,int> > windows;
  //windows.push_back(pair<int,int>( 64,  64));
  windows.push_back(pair<int,int>( 64, 128));
  // windows.push_back(pair<int,int>(128,  64));
  // windows.push_back(pair<int,int>(128, 128));

  for (int i = 0; i < count; i++)
  {
    std::stringstream filepath;
    //filepath << "/home/thermans/data/robot-frames/test1/" << i << ".png";
    filepath << "/home/thermans/data/robot.jpg";
    std::cout << "Image " << i << std::endl;
    Mat frame;
    frame = cv::imread(filepath.str());
    Mat bw_frame(frame.rows, frame.cols, CV_8UC1);
    cvtColor(frame, bw_frame, CV_RGB2GRAY);

    // cv::namedWindow("saliency map");
    // cv::namedWindow("raw img");

    try
    {
      //swd.scanImage(frame, windows);
      Mat disp_img = csm(frame);
      cv::waitKey();
    }
    catch(cv::Exception e)
    {
      std::cerr << e.err << std::endl;
    }
  }
  return 0;
}
