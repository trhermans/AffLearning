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
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <sstream>

using cv::Mat;
using std::vector;

CenterSurroundMapper::CenterSurroundMapper(int min_c, int max_c, int min_delta,
                                           int max_delta) :
    min_c_(min_c), max_c_(max_c), min_delta_(min_delta), max_delta_(max_delta),
    N_(4), gabor_size_(7)
{
  num_scales_ = max_c_ + max_delta_;
  generateGaborFilters();
}

void CenterSurroundMapper::generateGaborFilters()
{
  Mat base_x(gabor_size_, gabor_size_, CV_64FC1);
  Mat base_y(gabor_size_, gabor_size_, CV_64FC1);

  // Populate the base x and y matrices
  for (int i = 0; i < base_x.cols; ++i)
  {
    for (int j = 0; j < base_x.rows; ++j)
    {
      base_x.at<double>(i,j) = i - gabor_size_/2;
      base_y.at<double>(i,j) = j - gabor_size_/2;
    }
  }

  for (int alpha = 0; alpha < N_; ++alpha)
  {
    float theta = M_PI / N_*alpha;
    Mat x_theta = base_x *  cos(theta) + base_y * sin(theta);
    Mat y_theta = base_x * -sin(theta) + base_y * cos(theta);

    Mat gabor_alpha(gabor_size_, gabor_size_, CV_64FC1, 1.0);
    gabor_alpha *= 1/(2*M_PI);

    Mat to_exp(gabor_size_, gabor_size_, CV_64FC1);
    to_exp = (x_theta.mul(x_theta) + y_theta.mul(y_theta))*-0.5;

    for(unsigned int i = 0; i < gabor_size_; i++)
    {
      for(unsigned int j = 0; j < gabor_size_; j++)
      {
        gabor_alpha.at<double>(i,j) *= exp(to_exp.at<double>(i,j))*
            cos(x_theta.at<double>(i,j)*M_PI*2);
      }
    }

    gabor_filters_.push_back(gabor_alpha);
  }
}

Mat CenterSurroundMapper::operator()(Mat& frame, bool use_gradient)
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
  vector<vector<Mat> > O_n_theta; // First index scale, second index relative
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

  // NOTE: in the paper they do intensity as average of rgb, I'm using grayscale
  cv::buildPyramid(I, I_scales, num_scales_);
  cv::buildPyramid(R, R_scales, num_scales_);
  cv::buildPyramid(G, G_scales, num_scales_);
  cv::buildPyramid(B, B_scales, num_scales_);
  cv::buildPyramid(Y, Y_scales, num_scales_);

  //
  // Build Gabor orientation Pyramid
  //

  // Get laplacians at all scales (TODO: do this while building pyr via DoG)
  for(unsigned int i = 0; i < I_scales.size(); ++i)
  {
    Mat lap;
    cv::Laplacian(I_scales[i], lap, I_scales[i].depth());
    L_scales.push_back(lap);
    vector<Mat> O_theta;

    // Get the N orientation maps for each scale
    for (int i = 0; i < N_; i++)
    {
      Mat m_i(lap.rows, lap.cols, lap.type());
      cv::filter2D(lap, m_i, -1, gabor_filters_[i]);

      // For each of the N orientation maps smooth and downsample
      O_theta.push_back(m_i);
    }
    O_n_theta.push_back(O_theta);
  }

  //
  // Get multi-scale maps of the features
  //

  vector<Mat> I_cs;
  vector<Mat> RG_cs;
  vector<Mat> BY_cs;
  vector<Mat> C_cs;
  vector<vector<Mat> > O_theta_cs;


  // Initialize the vectors for the different orientations
  for (int a = 0; a < N_; ++a)
  {
    vector<Mat> O_cs;
    O_cs.clear();
    O_theta_cs.push_back(O_cs);
  }

  // Here we build the multi-scale maps
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

      // Build orientation maps for each of the N_ orientations
      for (int a = 0; a < N_; ++a)
      {
        O_theta_cs[a].push_back(mapDifference(O_n_theta[c][a],
                                              O_n_theta[s][a], c, s));
      }
    }
  }

  //
  // Normalize all maps, based on feature type
  //

  // Find max values to normalize maps by feature type
  int I_max = 0;
  int C_max = 0;
  vector<int> O_theta_max(N_, 0);

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

        for (int a = 0; a < N_; ++a)
        {
          if (O_theta_cs[a][i].at<uchar>(r,c) > O_theta_max[a])
            O_theta_max[a] = O_theta_cs[a][i].at<uchar>(r,c);
        }
      }
    }
  }

  // Perform the normalization
  for (unsigned int i = 0; i < I_cs.size(); ++i)
  {
    // Test for max value for normalization
    I_cs[i] = normalize(I_cs[i], I_max);
    C_cs.push_back(normalize(RG_cs[i], C_max) + normalize(BY_cs[i], C_max));
    for (int a = 0; a < N_; ++a)
    {
      O_theta_cs[a][i] = normalize(O_theta_cs[a][i], O_theta_max[a]);
    }
  }

  // Combine conspicuity maps into feature maps
  Mat I_bar;
  Mat C_bar;
  I_bar = mapSum(I_cs);
  C_bar = mapSum(C_cs);

  vector<Mat> O_bars;
  // For the orientations we sum each orientation separately, then combined
  // normalized forms of those four
  for (int a = 0; a < N_; ++a)
  {
    Mat O_bar_a = mapSum(O_theta_cs[a]);
    O_bars.push_back(O_bar_a);
    // std::stringstream img_name;
    // img_name << "O_bar_" << a;
    // cv::imshow(img_name.str(), O_bar_a);
  }
  //cv::waitKey();

  //
  // Normalize and sum the O_bars
  //

  Mat O_bar(I_bar.rows, I_bar.cols, I_bar.type(), 0);
  int O_max = 0;

  // Get the max of the o_bars
  for (int a = 0; a < N_; ++a)
  {
    for(int r = 0; r < O_bars[a].rows; ++r)
    {
      for(int c = 0; c < O_bars[a].cols; ++c)
      {
        if (O_bars[a].at<uchar>(r,c) > C_max)
          O_max = O_bars[a].at<uchar>(r,c);
      }
    }
  }

  for (int a = 0; a < N_; ++a)
  {
    O_bar += normalize(O_bars[a], O_max)*(1/float(N_));
  }

  //
  // Normalize and sum the different feature channels
  //

  // Get the max of the different feature channel conspicuicy maps
  int bar_max = 0;
  for (int r = 0; r < I_bar.rows; ++r)
  {
    for (int c = 0; c < I_bar.cols; ++c)
    {
      if (I_bar.at<uchar>(r,c) > bar_max)
        bar_max = I_bar.at<uchar>(r,c);
      if (C_bar.at<uchar>(r,c) > bar_max)
        bar_max = C_bar.at<uchar>(r,c);
      if (O_bar.at<uchar>(r,c) > bar_max)
        bar_max = O_bar.at<uchar>(r,c);
    }
  }

  std::cout << "bar max is: " << bar_max << std::endl;

  // Build the saliency map as the combination of the feature maps
  Mat saliency_map(I_bar.rows, I_bar.cols, CV_8UC1);
  saliency_map = (normalize(I_bar, bar_max)*(1/3.0) +
                  normalize(C_bar, bar_max)*(1/3.0) +
                  normalize(O_bar, bar_max)*(1/3.0));

  Mat gradient_map(I_bar.rows, I_bar.cols, CV_8UC1);
  if (use_gradient) // TODO: Make nicer functionality for specifying this map
  {
    saliency_map.copyTo(gradient_map);

    for (unsigned int i = 0; i < gradient_map.rows; ++i)
    {
      for (unsigned int j = 0; j < gradient_map.cols; ++j)
      {
        gradient_map.at<uchar>(i,j) = 255*i/gradient_map.rows;
      }
    }

    saliency_map = saliency_map*0.5 + gradient_map*0.5;
  }

  Mat scaled;
  cv::equalizeHist(saliency_map, scaled);
  // cv::imshow("I bar", I_bar);
  // cv::imshow("C bar", C_bar);
  // cv::imshow("O bar", O_bar);
  // if (use_gradient)
  //   cv::imshow("Top Down map", gradient_map);
  // cv::imshow("Saliency", saliency_map);
  // cv::imshow("Scaled", scaled);
  // cv::waitKey(2);

  return scaled;
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
  //std::cout << "Found " << maxima.size() << " local maxima." << std::endl;

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
