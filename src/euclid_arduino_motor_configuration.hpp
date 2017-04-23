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

#ifndef __ARDUINO_MOTOR_
#define __ARDUINO_MOTOR_
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Bool.h>

#include <euclid_arduino_wrapper/EuclidMotorConfigurationConfig.h>
    


namespace euclid_arduino_wrapper {
    class EuclidArduinoMotorConfiguration : public nodelet::Nodelet {
    
        ros::Publisher mForwardPub;
        ros::Publisher mBackwardPub;
        ros::Publisher mRotationPub;
        ros::Publisher mLeftGainPub;
        ros::Publisher mRightGainPub;

        int queue_size_;
        float mForwardMultiplier;
        float mBackwardMultiplier;
    	float mRotationMultiplier;
	float mLeftGain;
    	float mRightGain;
	bool mIsArduinoReady;	
	ros::Subscriber mArduinoStatusSub ;
	    	
        boost::shared_ptr<dynamic_reconfigure::Server<euclid_arduino_wrapper::EuclidMotorConfigurationConfig> > dynamic_reconf_server_;

	void arduinoStatusCallback(const std_msgs::Bool &msg);

	void publishData();

        public:
            virtual void onInit();
            
            void dynamicConfigureServerCallback(euclid_arduino_wrapper::EuclidMotorConfigurationConfig &config, uint32_t level);
            

    };





}

#endif //__ARDUINO_MOTOR_
