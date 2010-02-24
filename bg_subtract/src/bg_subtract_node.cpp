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

#include <ros/ros.h>
#include <opencv/cv.h>
#include "sensor_msgs/Image.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include "bg_subtract.h"
#include "bg_subtract/ContourArray.h"

/**
 * @file   bg_subtract_node.cpp
 * @author Tucker Hermans <thermans@cc.gatech.edu>
 * @date   Tue Feb 16 19:17:17 2010
 *
 * @brief  ROS node to provide background subtraction on an image stream
 *
 */
class BgSubtractNode
{
  public:
    // Constructors and Destructors
    BgSubtractNode(ros::NodeHandle &n) :
            n_(n), it_(n)
    {
        image_sub_ = it_.subscribe("image_topic", 1,
                                   &BgSubtractNode::imageCallback, this);
        contour_publisher_ = n.advertise<bg_subtract::ContourArray>("contours"
                                                                    ,1);
    }

    ~BgSubtractNode()
    {
    }

    // Publish and Subscribe methods
    void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        // Convert from ROS image to CV image
        IplImage* fg_img = NULL;
        try
        {
            fg_img = bridge_.imgMsgToCv(msg_ptr);
        }
        catch (sensor_msgs::CvBridgeException error)
        {
            ROS_ERROR("Error converting ROS image to IplImage");
        }

        cv::Mat fg_mat = fg_img;

        // Perform the background subtraction
        bg_gui_.updateDisplay(fg_mat);

        // Publish the generated contours
        bg_subtract::ContourArray contour_msg;
        std::vector<std::vector<cv::Point> > contours;
        contours = bg_gui_.bg_sub_.getContours();

        for (unsigned int i = 0; i < contours.size(); ++i) {
            for (unsigned int j = 0; j < contours[i].size(); ++j) {
                contour_msg.contours[i].points[j].x = contours[i][j].x;
                contour_msg.contours[i].points[j].y = contours[i][j].y;
            }
        }

        contour_publisher_.publish(contour_msg);
    }

  protected:
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher contour_publisher_;
    sensor_msgs::CvBridge bridge_;

    BgSubtractGUI bg_gui_;
};


/**
 * Method to start the background subtraction ros node
 */
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "bg_subtract");
    ros::NodeHandle n;
    BgSubtractNode bgs_node(n);
    ros::spin();
}

