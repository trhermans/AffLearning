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

CenterSurroundMapper::CenterSurroundMapper(int min_c, int max_c, int min_delta,
                                           int max_delta) :
    min_c_(min_c), max_c_(max_c), min_delta_(min_delta), max_delta_(max_delta),
    N_(4)
{
  num_scales_ = max_c_ + max_delta_;
  generateGaborFilters();
}

void generateGaborFilters()
{
  for (unsigned int alpha = 0; alpha < N_; ++alpha)
  {
    float theta = M_PI/N_*alpha;
    
  }
}

Mat CenterSurroundMapper::operator()(Mat& frame)
{
  Mat I(frame.rows, frame.cols, CV_8UC1);

  cvtColor(frame, I, CV_RGB2GRAY);

  Mat R(frame.rows, frame.cols, CV_8UC1);
  Mat G(frame.rows, frame.cols, CV_8UC1);
  Mat B(frame.rows, frame.cols, CV_8UC1);
  Mat Y(frame.rows, frame.cols, CV_8UC1);
  vector<Mat> I_scales;
  vector<Mat> R_scales;
  vector<Mat> G_scales;
  vector<Mat> B_scales;
  vector<Mat> Y_scales;
  vector<Mat> L_scales;
  vector<vector<Mat> > O_n_sigma; // First index scale, second index relative
                                  // orientation (default 0 - 3)

  // Get the component color channels
  vector<Mat> channels;
  split(frame, channels);

  // Get intensity independent hue channels
  R = channels[0] - (channels[1]/2.0 + channels[2]/2.0);
  G = channels[1] - (channels[0]/2.0 + channels[2]/2.0);
  B = channels[2] - (channels[0]/2.0 + channels[1]/2.0);
  Y = ((channels[0]/2.0 + channels[1]/2.0) -
       (channels[0]/2.0 - channels[1]/2.0)
       - channels[2]);

  // Get copies of the four feature maps at all scales
  cv::buildPyramid(I, I_scales, num_scales_);
  cv::buildPyramid(R, R_scales, num_scales_);
  cv::buildPyramid(G, G_scales, num_scales_);
  cv::buildPyramid(B, B_scales, num_scales_);
  cv::buildPyramid(Y, Y_scales, num_scales_);

  // Get multi-scale maps of the features
  vector<Mat> I_cs;
  vector<Mat> RG_cs;
  vector<Mat> BY_cs;
  vector<Mat> C_cs;
  vector<Mat> O_cs;

  for (int c = min_c_; c <= max_c_; c++)
  {
    for (int s = c + min_delta_; s <= c + max_delta_; s++)
    {
      // Build intensity maps
      I_cs.push_back(mapDifference(I_scales[c], I_scales[s], c, s));

      // Build red-green opponent color maps
      Mat RG_c = R_scales[c] - G_scales[c];
      Mat GR_s = G_scales[s] - R_scales[s];
      RG_cs.push_back(mapDifference(RG_c, GR_s, c, s));

      // Build blue-yellow opponent color maps
      Mat BY_c = B_scales[c] - Y_scales[c];
      Mat YB_s = Y_scales[s] - B_scales[s];
      BY_cs.push_back(mapDifference(BY_c, YB_s, c, s));
    }
  }

  //
  // Build Gabor orientation Pyramid
  //

  // Get laplacians at all scales (TODO: do this while building pyr via DoG)
  for(unsigned int i = 0; i < I_scales.size(); ++i)
  {
    Mat lap;
    cv::Laplacian(I_scales[i], lap, I_scales[i].depth());
    L_scales.push_back(lap);
    vector<Mat> O_sigma;

    // Get the N orientation maps for each scale
    for (int i = 0; i < N_; i++)
    {
      Mat lap_m_i(lap.rows, lap.cols, lap.type());
      lap.copyTo(lap_m_i);
      Mat m_i(lap_m_i.rows/2, lap_m_i.cols/2, lap_m_i.type());
      // For each of the N orientation maps smooth and downsample
      cv::pyrDown(lap_m_i, m_i);
    }
  }

  //
  // Normalize all maps, based on feature type
  //

  // Find max values to normalize maps by feature type
  int I_max = 0;
  int C_max = 0;
  for (unsigned int i = 0; i < I_cs.size(); ++i)
  {
    for(int r = 0; r < I_cs[i].rows; ++r)
    {
      for(int c = 0; c < I_cs[i].cols; ++c)
      {
        if (I_cs[i].at<uchar>(r,c) > I_max)
          I_max = I_cs[i].at<uchar>(r,c);
        if (RG_cs[i].at<uchar>(r,c) > C_max)
          C_max = RG_cs[i].at<uchar>(r,c);
        if (BY_cs[i].at<uchar>(r,c) > C_max)
          C_max = BY_cs[i].at<uchar>(r,c);

      }
    }
  }

  // Perform the normalization
  for (unsigned int i = 0; i < I_cs.size(); ++i)
  {
    // Test for max value for normalization
    I_cs[i] = normalize(I_cs[i], I_max);
    C_cs.push_back(normalize(RG_cs[i], C_max) + normalize(BY_cs[i], C_max));
  }

  // Combine conspicuity maps into feature maps
  Mat I_bar;
  Mat C_bar;
  Mat O_bar;
  I_bar = mapSum(I_cs);
  C_bar = mapSum(C_cs);

  int bar_max = 0;
  for (int r = 0; r < I_bar.rows; ++r)
  {
    for (int c = 0; c < I_bar.cols; ++c)
    {
      if (I_bar.at<uchar>(r,c) > bar_max)
        bar_max = I_bar.at<uchar>(r,c);
      if (C_bar.at<uchar>(r,c) > bar_max)
        bar_max = C_bar.at<uchar>(r,c);
    }
  }

  std::cout << "bar max is: " << bar_max << std::endl;
  // Build the saliency map as the combination of the feature maps
  Mat saliency_map(I_bar.rows, I_bar.cols, CV_8UC1);
  saliency_map = normalize(I_bar, bar_max) + normalize(C_bar, bar_max);
  saliency_map /= 2;

  Mat scaled;
  cv::equalizeHist(saliency_map, scaled);
  cv::imshow("I bar", I_bar);
  cv::imshow("C bar", C_bar);
  cv::imshow("Saliency", saliency_map);
  cv::imshow("Scaled", scaled);
  cv::waitKey();


  return saliency_map;
}

Mat CenterSurroundMapper::mapDifference(Mat& m_c, Mat& m_s, int c, int s)
{
  // Upsample c to s resolution
  Mat m_s_prime(m_s.rows, m_s.cols, CV_8UC1);
  Mat temp;
  m_s.copyTo(temp);

  // Calculate the correct sizes to up sample to
  vector<cv::Size> sizes;
  cv::Size current_size(m_c.cols, m_c.rows);
  for (int i = c; i < s; i++)
  {
    sizes.push_back(current_size);
    current_size.width /= 2;
    current_size.height /= 2;
  }

  for (int i = c; i < s; i++)
  {
    cv::Size up_size;
    up_size = sizes.back();
    sizes.pop_back();
    cv::pyrUp(temp, m_s_prime, up_size);
    temp = m_s_prime;
  }

  // Take pixelwise difference
  Mat diff = abs(m_c - m_s_prime);

  return diff;
}

Mat CenterSurroundMapper::mapSum(vector<Mat>& maps)
{
  int min_rows = 10000;
  int min_cols = 10000;

  // Find the smallest scale of the images
  for (unsigned int i = 0; i < maps.size(); ++i)
  {
    if (maps[i].rows < min_rows)
    {
      min_rows = maps[i].rows;
      min_cols = maps[i].cols;
    }
  }

  Mat sum = Mat::zeros(min_rows, min_cols, CV_8UC1);

  for (unsigned int i = 0; i < maps.size(); ++i)
  {
    int num_steps = maps[i].cols / min_cols / 2;
    Mat m_prime = maps[i];
    Mat temp = maps[i];

    for (int j = 0; j < num_steps; ++j)
    {
      cv::pyrDown(temp, m_prime);
      temp = m_prime;
    }

    sum += m_prime;
  }

  return sum;
}

Mat CenterSurroundMapper::normalize(Mat& map, int M)
{
  // Normalize the values to the range 0 to max_val...
  int cur_max_val = 0;
  for(int r = 0; r < map.rows; ++r)
  {
    for (int c = 0; c < map.cols; ++c)
    {
      if (map.at<uchar>(r,c) > cur_max_val)
        cur_max_val = map.at<uchar>(r,c);
    }
  }

  Mat normalized = map * (M/float(cur_max_val));
  float thresh = M*0.20;

  // Find the local maxima
  vector<uchar> maxima;
  for(int r = 0; r < map.rows; ++r)
  {
    for(int c = 0; c < map.cols; ++c)
    {
      int val = map.at<uchar>(r,c);
      if (val > thresh)
      {
        // Test if maximal over the 3 by 3 window
        if (c > 0) // Has left
        {
          if( val < map.at<uchar>(r,c-1))
            continue;
          if (r > 0 && val < map.at<uchar>(r - 1, c - 1))
            continue;
          if (r < map.rows - 1 && val < map.at<uchar>(r + 1, c - 1))
            continue;
        }
        if (c < map.cols - 1) // Has Right
        {
          if( val < map.at<uchar>(r,c + 1))
            continue;
          if (r > 0 && val < map.at<uchar>(r - 1, c + 1))
            continue;
          if (r < map.rows - 1 && val < map.at<uchar>(r + 1, c + 1))
            continue;
        }
        if (r > 0 && val < map.at<uchar>(r - 1, c)) // Has above
          continue;
        if (r < map.rows - 1 && val < map.at<uchar>(r + 1, c)) // Has below
          continue;

        // Store the local maxima value
        maxima.push_back(val);
      }
    }
  }
  std::cout << "Found " << maxima.size() << " local maxima." << std::endl;

  // Get mean of the local maxima
  float m_bar = 0;

  for (unsigned int i = 0; i < maxima.size(); ++i)
  {
    m_bar += maxima[i];
  }

  m_bar /= maxima.size();

  // Finally perform the normalization based on the difference between the
  // maximum value and the mean of the local maxima
  float fact = (M - m_bar)*(M - m_bar);

  // Scale it to be within the image range
  fact *= 255/(fact*M);
  normalized *= fact;
  return normalized;
}

Mat CenterSurroundMapper::getOrientationMap(Mat& img, float theta)
{
  return img;
}
