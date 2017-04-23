/******************************************************************************
Copyright (c) 2016, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <pluginlib/class_list_macros.h>
#include "euclid_arduino_wrapper.hpp"

PLUGINLIB_EXPORT_CLASS(euclid_arduino_wrapper::EuclidArduinoWrapper, nodelet::Nodelet) 

namespace euclid_arduino_wrapper {
    
    void EuclidArduinoWrapper::onInit() {
        NODELET_DEBUG("Initializing nodelet...");
        roi_x = 0;
        roi_y = 0;
        roi_width = 10;
        roi_height = 10;
        ros::NodeHandle &nh         = getNodeHandle();
        ros::NodeHandle &private_nh = getPrivateNodeHandle();
        it_.reset(new image_transport::ImageTransport(nh));


        // Make sure we don't enter connectCb() between advertising and assigning to pub_rect_
        boost::lock_guard<boost::mutex> lock(connect_mutex_);
    
        // setting up dynamic reconfigure server
         dynamic_reconf_server_.reset(new dynamic_reconfigure::Server<euclid_arduino_wrapper::EuclidArduinoWrapperConfig>(getPrivateNodeHandle()));
         dynamic_reconf_server_->setCallback(boost::bind(&EuclidArduinoWrapper::dynamicConfigureServerCallback,this, _1, _2));
         sub_depth_ = it_->subscribe("camera/depth/image_raw", 1, &EuclidArduinoWrapper::imageCb, this);
    
    }
    void EuclidArduinoWrapper::connectCb() {
    
        boost::lock_guard<boost::mutex> lock(connect_mutex_);
        if (pub_arduino_.getNumSubscribers() == 0)
            sub_depth_.shutdown();
        else if (!sub_depth_)
        {

            sub_depth_ = it_->subscribe("camera/depth/image_raw", 1, &EuclidArduinoWrapper::imageCb, this);
        }
    }

    void EuclidArduinoWrapper::imageCb(const sensor_msgs::ImageConstPtr& image_msg)
    {

      // Create cv::Mat views onto both buffers
      cv_bridge::CvImagePtr originalMsg = cv_bridge::toCvCopy(image_msg,"");
      const cv::Mat image = originalMsg->image;
      cv::Rect rec(roi_x, roi_y, roi_width,roi_height);

      cv::Mat roi = image(rec);
 
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), image_msg->encoding, roi).toImageMsg();
      pub_arduino_.publish(msg);
    }
    
    void EuclidArduinoWrapper::dynamicConfigureServerCallback(euclid_arduino_wrapper::EuclidArduinoWrapperConfig &config, uint32_t level) {
        roi_x = config.depth_roi_x;
        roi_y = config.depth_roi_y;
        roi_width = config.depth_roi_width;
        roi_height= config.depth_roi_height;
    }


}

