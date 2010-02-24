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
#include <bg_subtract/ContourArray.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

#include "overhead_tracking.h"

class OverheadTrackingNode
{
  public:
    // Constructors and Destructors
    OverheadTrackingNode(ros::NodeHandle &n) :
            n_(n), it_(n), tracker_("Contour Window")
    {
        image_sub_ = it_.subscribe("image_topic", 1,
                                   &OverheadTrackingNode::imageCallback, this);

        contour_subscriber_ = n.subscribe("contours", 1,
                                          &OverheadTrackingNode::
                                          contourCallback, this);
    }

    // Publish and Subscribe methods

    /**
     * Callback method to the image transport subscriber. Currently only saves a
     * copy of the most recent image internally to the node for future use.
     *
     * @param msg_ptr Most recent image off of the topic
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg_ptr)
    {
        IplImage* fg_img = NULL;
        try
        {
            fg_img = bridge_.imgMsgToCv(msg_ptr);
        }
        catch (sensor_msgs::CvBridgeException error)
        {
            ROS_ERROR("Error converting ROS image to IplImage");
        }

        // Save a copy of the image
        cv::Mat fg_mat = fg_img;
        fg_mat.copyTo(current_img_);
    }

    /**
     * Callback method for the contour subscriber.  Draws the contours to the
     * display and extracts information from them.
     *
     * @param contour_msg most recent contours pushed to the topic
     */
    void contourCallback(const bg_subtract::ContourArrayConstPtr& contour_msg)
    {
        // Convert the message into opencv contour language
        contours_.clear();

        for (unsigned int i = 0; i < contour_msg->contours.size(); ++i)
        {
            std::vector<cv::Point> contour;
            for (unsigned int j = 0;
                 j < contour_msg->contours[i].points.size();
                 ++j)
            {
                int x = contour_msg->contours[i].points[j].x;
                int y = contour_msg->contours[i].points[j].y;
                cv::Point pt(x,y);
                contour.push_back(pt);
            }
            contours_.push_back(contour);
        }

        // Update our contour image
        tracker_.updateDisplay(current_img_, contours_);
    }

  protected:
    ros::NodeHandle n_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    sensor_msgs::CvBridge bridge_;
    ros::Subscriber contour_subscriber_;

    OverheadTracker tracker_;
    cv::Mat current_img_;
    std::vector<std::vector<cv::Point> > contours_;
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "overhead_tracking");
    ros::NodeHandle n;
    OverheadTrackingNode overhead_node(n);
    ros::spin();
}

