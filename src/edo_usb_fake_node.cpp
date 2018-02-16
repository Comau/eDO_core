/*
  Copyright (c) 2017, COMAU S.p.A.
  All rights reserved.
  
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  
  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
  ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  
  The views and conclusions contained in the software and documentation are those
  of the authors and should not be interpreted as representing official policies,
  either expressed or implied, of the FreeBSD Project.
*/
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "edo_core_msgs/JointControlArray.h"
#include "edo_core_msgs/JointStateArray.h"
#include "edo_core_msgs/JointCalibration.h"
#include "edo_core_msgs/JointConfigurationArray.h"
#include "edo_core_msgs/JointInit.h"
#include "edo_core_msgs/JointReset.h"
#include "CommonService.h"
#include "State.h"
#include <std_msgs/UInt8.h>
#include <iostream>
#include <cstdio>

#define TIMES_COMMAND_FLAG_HIGH 8

// -------------- TOPIC FROM/TO USB/CAN MODULE --------------
// Topics for command CALIBRATE / RESET / CONFIGURATION
// This is the topic where the node resets the joints
ros::Subscriber machine_jnt_reset_subscriber;
// This is the topic where the node configures the joints
ros::Subscriber machine_jnt_config_subscriber;
// This is the topic where the node calibrates the joints
ros::Subscriber machine_jnt_calib_subscriber;
// This is the topic where the node publishes the init commands
ros::Subscriber machine_jnt_init_subscriber;
// This is the topic where the node publishes the firmware version request commands
ros::Subscriber machine_jnt_version_subscriber;
// This is the topic where the node sends the joints state
ros::Publisher usb_jnt_state_publisher;
// This is the topic where the usb node sends the joints firmware version
ros::Publisher usb_jnt_version_publisher;
// This is the topic where the node subscribes to control commands (from Algorithm node)
ros::Subscriber machine_jnt_ctrl_subscriber;

edo_core_msgs::JointStateArray state_msg;
bool reset_commandflag = false;
int numberOfJoints = 0;

void setCommandFlag(COMMAND_FLAG ack_type, uint64_t mask)
{
	for (unsigned int i = 0; i < numberOfJoints; i++) {
		if ((mask & (1 << i)) == (1 << i)) {
			state_msg.joints[i].commandFlag |= ack_type;
			reset_commandflag = true;
		}
	}
}

void resetCommandFlag()
{
	for (unsigned int i = 0; i < numberOfJoints; i++) {
		state_msg.joints[i].commandFlag &= 0xF0;
		reset_commandflag = false;
	}
}

void ResetReceived(const edo_core_msgs::JointReset msg)
{
	ROS_INFO("JointReset received: mask [%llu]", msg.joints_mask);
	setCommandFlag(COMMAND_FLAG::ACK_RESET, msg.joints_mask);
}

void ConfigReceived(const edo_core_msgs::JointConfigurationArray msg)
{
	ROS_INFO("JointConfig received: mask [%llu]", msg.joints_mask);
	setCommandFlag(COMMAND_FLAG::ACK_CONFIG, msg.joints_mask);
}

void InitReceived(const edo_core_msgs::JointInit msg) {
	ROS_INFO("JointInit received: mask [%llu]", msg.joints_mask);

	if(msg.mode == 0){
		int res = 0;
		uint64_t mask = msg.joints_mask;
		uint8_t bit1 = 0x01;	
	
		for(uint8_t i = 0; i < 64; i++)
		{
			if( (mask & bit1) == bit1)
				res++;
			mask = mask >> 1;
		}
		numberOfJoints = res;
		state_msg.joints.resize(numberOfJoints);
		state_msg.joints_mask = pow(2, numberOfJoints) - 1;
	}
	
	setCommandFlag(COMMAND_FLAG::ACK_INIT, msg.joints_mask);
}

void CalibrationReceived(const edo_core_msgs::JointCalibration msg) {
	ROS_INFO("JointCalibration received: mask [%llu]", msg.joints_mask);
	setCommandFlag(COMMAND_FLAG::ACK_CALIBRATION, msg.joints_mask);
}

void VersionReceived(const std_msgs::UInt8 msg) {
	ROS_INFO("JointVersion request received for: %d (%s)", msg.data, (msg.data == 0 ? "usb" : "joint"));
	
	edo_core_msgs::JointFwVersion _jnt_fw_version_pub_msg;
	_jnt_fw_version_pub_msg.id = msg.data;
	_jnt_fw_version_pub_msg.majorRev = 2;
	_jnt_fw_version_pub_msg.minorRev = 0;
	_jnt_fw_version_pub_msg.revision = 802;
	_jnt_fw_version_pub_msg.svn = 412;
	
	ROS_INFO("Publish firmware version for %d", msg.data);
	usb_jnt_version_publisher.publish(_jnt_fw_version_pub_msg);
}

void AlgoJointControl(const edo_core_msgs::JointControlArray msg) {

	if(numberOfJoints != msg.joints.size())
		return;

	for (unsigned int jointPos = 0; jointPos < msg.joints.size(); jointPos++) {
		state_msg.joints[jointPos].position = msg.joints[jointPos].position;
		state_msg.joints[jointPos].velocity = msg.joints[jointPos].velocity;
		state_msg.joints[jointPos].current = msg.joints[jointPos].current;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv,"edo_usb_fake");
	ros::NodeHandle node_obj;
	ros::Rate loop_rate(100);
	
	int countCommandFlagHigh = 0;

	machine_jnt_reset_subscriber = node_obj.subscribe("/machine_jnt_reset",10, &ResetReceived);
	machine_jnt_config_subscriber = node_obj.subscribe("/machine_jnt_config",10, &ConfigReceived);
	machine_jnt_calib_subscriber = node_obj.subscribe("/machine_jnt_calib",10, &CalibrationReceived);
	machine_jnt_init_subscriber = node_obj.subscribe("/machine_init",10, &InitReceived);
	machine_jnt_version_subscriber = node_obj.subscribe("/machine_jnt_version",10, &VersionReceived);
	usb_jnt_state_publisher = node_obj.advertise<edo_core_msgs::JointStateArray>("/usb_jnt_state",10);
	usb_jnt_version_publisher = node_obj.advertise<edo_core_msgs::JointFwVersion>("/usb_jnt_version",10);
	machine_jnt_ctrl_subscriber = node_obj.subscribe("/algo_jnt_ctrl",10, &AlgoJointControl);

	state_msg.joints.resize(numberOfJoints);
	state_msg.joints_mask = pow(2, numberOfJoints) - 1;
	for (unsigned int jointPos = 0; jointPos < state_msg.joints.size(); jointPos++) {
		state_msg.joints[jointPos].position = 0;
		state_msg.joints[jointPos].velocity = 0;
		state_msg.joints[jointPos].current = 0.15;
		state_msg.joints[jointPos].commandFlag = 0;
	}
	
	while(ros::ok())
	{
	      ros::spinOnce();
	      
	      if(reset_commandflag){
	      	countCommandFlagHigh++;
	      	if(countCommandFlagHigh == TIMES_COMMAND_FLAG_HIGH){
	      		resetCommandFlag();
	      		countCommandFlagHigh = 0;
	      	}
	      }
	      usb_jnt_state_publisher.publish(state_msg);
	      
	      
	      loop_rate.sleep();
	}

	return 0;
}
