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

#define TIMES_COMMAND_FLAG_HIGH 30
#define AXIS_SEVEN_BIT_MASK     ((uint32_t)0x00000040)
#define AXIS_SEVEN              6  /* 0..6 */
#define NIBBLE_ERROR_MASK       0xF0

enum INIT_MODE {
	DISCOVERY_MODE = 0,
	SET_MODE,
	CANCEL_MODE
};

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

edo_core_msgs::JointStateArray _state_msg;
bool         _glb_reset_commandflag = false;
unsigned int _glb_numberOfJoints    = 0;
uint32_t     _glb_joints_mask       = 0;

void setCommandFlag(COMMAND_FLAG ack_type, uint32_t joints_mask)
{
  uint32_t mask = joints_mask;

  if (mask != 0)
  {
    for(unsigned int jnt_idx = 0; mask != 0; jnt_idx++, mask = mask >> 1)
    {
      if(mask & 1)
      {
        /* In the first nibble, the least significant one, there is the code of the command. */
        /* In the second nibble, the most significant one, each bit has a special meaning and it is an error bit */
        _state_msg.joints[jnt_idx].commandFlag |= ack_type;
        _glb_reset_commandflag = true;
      }
    }
  }
}

void resetCommandFlag()
{
  uint32_t mask = _glb_joints_mask;
  
  if (mask != 0)
  {
    for(unsigned int jnt_idx = 0; mask != 0; jnt_idx++, mask = mask >> 1)
    {
      if(mask & 1)
      {
        /* In the first nibble, the least significant one, there is the code of the command. */
        /* In the second nibble, the most significant one, each bit has a special meaning and it is an error bit */
        _state_msg.joints[jnt_idx].commandFlag &= NIBBLE_ERROR_MASK;
        _glb_reset_commandflag = false;
      }
    }
  }
}

void ResetReceived(const edo_core_msgs::JointReset msg)
{
  uint32_t mask = (uint32_t)msg.joints_mask;
  
  ROS_INFO("JointReset received: mask [%lu]", mask);
  for(unsigned int jnt_idx = 0; mask != 0; jnt_idx++, mask = mask >> 1)
  {
  	if(mask & 1)
    {
      _state_msg.joints[jnt_idx].position = 0;
      _state_msg.joints[jnt_idx].velocity = 0;
      _state_msg.joints[jnt_idx].current = 0.15;
      /* In the most significant nibble, the state is a bit mask. */
      _state_msg.joints[jnt_idx].commandFlag &= ~(1 << COMMAND_FLAG::H_BRIDGE_DOWN);
    }
  }
  setCommandFlag(COMMAND_FLAG::ACK_RESET, (uint32_t)msg.joints_mask);
}

void ConfigReceived(const edo_core_msgs::JointConfigurationArray msg)
{
  uint32_t mask = (uint32_t)msg.joints_mask;
  
  ROS_INFO("JointConfig received: mask [%lu]", mask);
  setCommandFlag(COMMAND_FLAG::ACK_CONFIG, mask);
}

void InitReceived(const edo_core_msgs::JointInit msg)
{
  uint32_t mask;

  mask = (uint32_t)msg.joints_mask;
  ROS_INFO("JointInit received: mask [%u] mode [%d]", mask, msg.mode);

  if(msg.mode == INIT_MODE::DISCOVERY_MODE)
  {
    unsigned int jnt_idx, last_jnt, i;
    
    for(i = 0, jnt_idx = 0, last_jnt = 0; mask != 0; i++, mask = mask >> 1)
    {
      /* Does this joint exist? */
      if(mask & 1)
      { /* Yes */
        last_jnt = i+1; /* This joint is a good candidate to be the last */
        jnt_idx++;      /* Update the joint counter */
      }
    }
    /*
    * Now I can have:
    *    a robot with a joint_mask of 0x7F, so last_jnt = 7 and jnt_idx = 7
    *    a robot with a joint_mask of 0x3F, so last_jnt = 6 and jnt_idx = 6
    *    a robot with a joint_mask of 0x57, so last_jnt = 7 and jnt_idx = 5 because joint four and six are missing
    *    a robot with a joint_mask of 0x17, so last_jnt = 5 and jnt_idx = 4 because joint four and six are missing
    *
    * In order to have enough space also for the last axis, I use the 'last_jnt' to resize the buffer.
    */
    if ((_state_msg.joints.size() != last_jnt) && (jnt_idx != 0))
    {
      _state_msg.joints.resize(last_jnt);
      _state_msg.joints_mask = msg.joints_mask;
      mask = (uint32_t)_state_msg.joints_mask;
      for (jnt_idx = 0; mask != 0; jnt_idx++, mask = mask >> 1)
      {
        if(mask & 1)
        {
          _state_msg.joints[jnt_idx].position = 0;
          _state_msg.joints[jnt_idx].velocity = 0;
          _state_msg.joints[jnt_idx].current = 0;
          /* In the most significant nibble, the state is a bit mask. */
          if (jnt_idx != AXIS_SEVEN)
            _state_msg.joints[jnt_idx].commandFlag = ((1 << COMMAND_FLAG::UNCALIBRATED) | (1 << COMMAND_FLAG::H_BRIDGE_DOWN));
          else
            _state_msg.joints[jnt_idx].commandFlag = 0;          
        }
      }
      _glb_joints_mask = (uint32_t)msg.joints_mask;
      _glb_numberOfJoints = last_jnt;
    }
  }

  setCommandFlag(COMMAND_FLAG::ACK_INIT, (uint32_t)msg.joints_mask);
}

void CalibrationReceived(const edo_core_msgs::JointCalibration msg)
{
  uint32_t mask = (uint32_t)msg.joints_mask;

  ROS_INFO("JointCalibration received: mask [%lu]", mask);
  for(unsigned int jnt_idx = 0; mask != 0; jnt_idx++, mask = mask >> 1)
  {
    if(mask & 1)
    {
      _state_msg.joints[jnt_idx].position = 0;
      _state_msg.joints[jnt_idx].velocity = 0;
      _state_msg.joints[jnt_idx].current = 0.15;
      /* In the most significant nibble, the state is a bit mask. */
      _state_msg.joints[jnt_idx].commandFlag &= ~(1 << COMMAND_FLAG::UNCALIBRATED);
    }
  }
  setCommandFlag(COMMAND_FLAG::ACK_CALIBRATION, (uint32_t)msg.joints_mask);
}

void VersionReceived(const std_msgs::UInt8 msg) {
  ROS_INFO("JointVersion request received for: %d (%s)", msg.data, (msg.data == 0 ? "usb" : "joint"));
  
  edo_core_msgs::JointFwVersion _jnt_fw_version_pub_msg;
  _jnt_fw_version_pub_msg.id = msg.data;
  _jnt_fw_version_pub_msg.majorRev = 2;
  _jnt_fw_version_pub_msg.minorRev = 0;
  _jnt_fw_version_pub_msg.revision = 999;
  _jnt_fw_version_pub_msg.svn = 999;
  
  ROS_INFO("Publish firmware version for %d", msg.data);
  usb_jnt_version_publisher.publish(_jnt_fw_version_pub_msg);
}

void AlgoJointControl(const edo_core_msgs::JointControlArray msg)
{
  uint32_t mask = _glb_joints_mask;

  if(_glb_numberOfJoints == 0)
    return;
  if(_glb_numberOfJoints != msg.joints.size())
    return;

  for(unsigned int jnt_idx = 0; mask != 0; jnt_idx++, mask = mask >> 1)
  {
    if(mask & 1)
    {
      _state_msg.joints[jnt_idx].position = msg.joints[jnt_idx].position;
      _state_msg.joints[jnt_idx].velocity = msg.joints[jnt_idx].velocity;
      _state_msg.joints[jnt_idx].current = msg.joints[jnt_idx].current;
    }
    else
    {
      _state_msg.joints[jnt_idx].position = 0;
      _state_msg.joints[jnt_idx].velocity = 0;
      _state_msg.joints[jnt_idx].current = 0;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv,"edo_usb_fake");
  ros::NodeHandle node_obj;
  ros::Rate loop_rate(100);  /* Every 10 ms */
  
  int countCommandFlagHigh = 0;
  
  machine_jnt_reset_subscriber = node_obj.subscribe("/machine_jnt_reset",10, &ResetReceived);
  machine_jnt_config_subscriber = node_obj.subscribe("/machine_jnt_config",10, &ConfigReceived);
  machine_jnt_calib_subscriber = node_obj.subscribe("/machine_jnt_calib",10, &CalibrationReceived);
  machine_jnt_init_subscriber = node_obj.subscribe("/machine_init",10, &InitReceived);
  machine_jnt_version_subscriber = node_obj.subscribe("/machine_jnt_version",10, &VersionReceived);
  usb_jnt_state_publisher = node_obj.advertise<edo_core_msgs::JointStateArray>("/usb_jnt_state",10);
  usb_jnt_version_publisher = node_obj.advertise<edo_core_msgs::JointFwVersion>("/usb_jnt_version",10);
  machine_jnt_ctrl_subscriber = node_obj.subscribe("/algo_jnt_ctrl",10, &AlgoJointControl);
  
  if (_glb_numberOfJoints != 0)
  {
    _state_msg.joints.resize(_glb_numberOfJoints);
    _state_msg.joints_mask = pow(2, _glb_numberOfJoints) - 1;
    for (unsigned int jnt_idx = 0; jnt_idx < _state_msg.joints.size(); jnt_idx++) {
      _state_msg.joints[jnt_idx].position = 0;
      _state_msg.joints[jnt_idx].velocity = 0;
      _state_msg.joints[jnt_idx].current = 0;
      /* In the most significant nibble, the state is a bit mask. */
      if (jnt_idx != AXIS_SEVEN)
        _state_msg.joints[jnt_idx].commandFlag = ((1 << COMMAND_FLAG::UNCALIBRATED) | (1 << COMMAND_FLAG::H_BRIDGE_DOWN));
      else
        _state_msg.joints[jnt_idx].commandFlag = 0;
    }
  }
  while(ros::ok())
  {
    ros::spinOnce();

    if(_glb_reset_commandflag)
    {
      countCommandFlagHigh++;
      if(countCommandFlagHigh == TIMES_COMMAND_FLAG_HIGH)
      {
        resetCommandFlag();
        countCommandFlagHigh = 0;
      }
    }
    usb_jnt_state_publisher.publish(_state_msg);

    loop_rate.sleep();
  }

  return 0;
}
