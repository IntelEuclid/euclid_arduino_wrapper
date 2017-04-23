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
#include "euclid_arduino_motor_configuration.hpp"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

PLUGINLIB_EXPORT_CLASS(euclid_arduino_wrapper::EuclidArduinoMotorConfiguration, nodelet::Nodelet) 

namespace euclid_arduino_wrapper {


    void EuclidArduinoMotorConfiguration::onInit() {
        NODELET_DEBUG("Initializing nodelet...");
        mForwardMultiplier = 1;
        mBackwardMultiplier = 1;
        mRotationMultiplier = 1;
        mLeftGain = 1;
        mRightGain = 1;

        ros::NodeHandle &nh         = getNodeHandle();
        ros::NodeHandle &private_nh = getPrivateNodeHandle();
        
        mForwardPub = nh.advertise<std_msgs::Float32>("/set_forward_multiply",1);
        mBackwardPub = nh.advertise<std_msgs::Float32>("/set_backward_multiply",1);
        mRotationPub = nh.advertise<std_msgs::Float32>("/set_rotation_multiply",1);
        mLeftGainPub = nh.advertise<std_msgs::Float32>("/set_left_gain",1);
        mRightGainPub = nh.advertise<std_msgs::Float32>("/set_right_gain",1);

        //setup the subscriber, arduino publishes when its ready
	mIsArduinoReady = false;
        mArduinoStatusSub = nh.subscribe("/arduino_ready", 1, &EuclidArduinoMotorConfiguration::arduinoStatusCallback, this);
     
        // setting up dynamic reconfigure server
        dynamic_reconf_server_.reset(new dynamic_reconfigure::Server<euclid_arduino_wrapper::EuclidMotorConfigurationConfig>(getPrivateNodeHandle()));
        dynamic_reconf_server_->setCallback(boost::bind(&EuclidArduinoMotorConfiguration::dynamicConfigureServerCallback,this, _1, _2));
    }
    
    void EuclidArduinoMotorConfiguration::dynamicConfigureServerCallback(euclid_arduino_wrapper::EuclidMotorConfigurationConfig &config, uint32_t level) {
        mBackwardMultiplier = config.backward_mult;
	mForwardMultiplier = config.forward_mult;
	mRotationMultiplier = config.rotation_mult;
	mLeftGain = config.left_gain;
	mRightGain = config.right_gain;

	publishData();
    }

    void EuclidArduinoMotorConfiguration::arduinoStatusCallback(const std_msgs::Bool &msg)
    {
	mIsArduinoReady = msg.data;
	publishData();
    }

    void EuclidArduinoMotorConfiguration::publishData() 
    {
	std_msgs::Float32 msg;

	msg.data = mBackwardMultiplier;
	mBackwardPub.publish(msg);

	msg.data = mForwardMultiplier;
	mForwardPub.publish(msg);
	      
	msg.data = mRotationMultiplier;
	mRotationPub.publish(msg);

	msg.data = mLeftGain;
	mLeftGainPub.publish(msg);  

	msg.data = mRightGain;
	mRightGainPub.publish(msg);
    }

}

