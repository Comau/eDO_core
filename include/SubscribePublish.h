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
/*
 * SubscribePublish.h
 *
 *  Created on: Jun 30, 2017
 *      Author: comau
 */

#ifndef EDO_CORE_PKG_SRC_SUBSCRIBEPUBLISH_H_
#define EDO_CORE_PKG_SRC_SUBSCRIBEPUBLISH_H_

#include "ros/ros.h"

#include "edo_core_msgs/JointControlArray.h"
#include "edo_core_msgs/JointStateArray.h"
#include "edo_core_msgs/JointCalibration.h"
#include "edo_core_msgs/MovementCommand.h"
#include "edo_core_msgs/MovementFeedback.h"
#include "edo_core_msgs/JointConfigurationArray.h"
#include "edo_core_msgs/JointsNumber.h"
#include "edo_core_msgs/JointReset.h"
#include "edo_core_msgs/JointInit.h"
#include "edo_core_msgs/MachineState.h"
#include "edo_core_msgs/NodeSwVersionArray.h"
#include <std_msgs/UInt8.h>

#include "StateManager.h"

class SubscribePublish
{
private:

    ros::NodeHandle node_obj;
    ros::Timer machineStateTimer;
    void timerMachineStateCallback(const ros::TimerEvent& event);

  	// -------------- TOPIC FROM/TO BRIDGE NODE --------------
	// This is the topic where the bridge node publishes reset command
	ros::Subscriber bridge_jnt_reset_subscriber;
	// This is the topic where the bridge node publishes configuration command
	ros::Subscriber bridge_jnt_config_subscriber;
	// This is the topic where the bridge node publishes calibration command
	ros::Subscriber bridge_jnt_calib_subscriber;
	// This is the topic where the bridge node publishes move commands
	ros::Subscriber bridge_move_subscriber;
	// This is the topic where the bridge node publishes jog commands
	ros::Subscriber bridge_jog_subscriber;
	// This is the topic where the bridge node publishes init commands
	ros::Subscriber bridge_init_subscriber;
	// This is the service where the node publishes the joints state
  	ros::Publisher machine_bridge_jnt_state_publisher;
	// This is the service where the node publishes the its state
  	ros::Publisher machine_state_publisher;
  	// This is the topic where the node publishes the move command ack
  	ros::Publisher machine_move_ack_publisher;
  	// This is the service server where the node sends the software version
	ros::ServiceServer bridge_sw_version_server;

	// -------------- TOPIC FROM/TO ALGORITHM NODE --------------
	// This is the service where the node receives the number of joints presents in the system
  	ros::ServiceClient algo_jnt_number_client;
  	// This is the service where the node receives the movement feedback
  	ros::Subscriber algo_move_ack_subscriber;
	// This is the service where the node publishes the joints state
  	ros::Publisher machine_algo_jnt_state_publisher;
	// This is the topic where the node publishes the movement commands
	ros::Publisher machine_move_publisher;
	// This is the topic where the node publishes the jog commands
	ros::Publisher machine_jog_publisher;

	// -------------- TOPIC FROM/TO USB/CAN MODULE --------------
	// Topics for command CALIBRATE / RESET / CONFIGURATION
	// This is the topic where the node resets the joints
	ros::Publisher machine_jnt_reset_publisher;
	// This is the topic where the node configures the joints
	ros::Publisher machine_jnt_config_publisher;
	// This is the topic where the node calibrates the joints
	ros::Publisher machine_jnt_calib_publisher;
	// This is the topic where the node publishes the init commands
	ros::Publisher machine_init_publisher;
	// This is the topic where the node sends the joints state
	ros::Subscriber usb_jnt_state_subscriber;
	// This is the topic where the usb node sends error
	ros::Subscriber usb_jnt_version_subscriber;
  	// This is the topic where the node publishes the software version
  	ros::Publisher machine_jnt_version_publisher;
	
	// -------------- TOPIC FROM RECOVERY NODE --------------
	ros::Subscriber recovery_edo_error_subscriber;
	

	StateManager manager;

	SubscribePublish();
	SubscribePublish(SubscribePublish const&);
	SubscribePublish& operator=(SubscribePublish const&);

	static SubscribePublish* instance;

	int jointsNumber;
	uint64_t maskedJoints;

public:

	// Singleton getInstance
	static SubscribePublish* getInstance();

	void CalibrationMsg(edo_core_msgs::JointCalibration msg);
	void ResetMsg(edo_core_msgs::JointReset msg);
	void ConfigureMsg(edo_core_msgs::JointConfigurationArray msg);
	void InitMsg(edo_core_msgs::JointInit msg);
	void JogMsg(const edo_core_msgs::MovementCommand& msg);
	void MoveMsg(const edo_core_msgs::MovementCommand& msg);
	void MoveAck(const edo_core_msgs::MovementFeedback& ack);
	void AlgorithmStatusMsg(const edo_core_msgs::JointStateArray& msg);
	void BridgeStatusMsg(const edo_core_msgs::JointStateArray& msg);
	void MachineStateMsg(const edo_core_msgs::MachineState& msg);
	void MachineSwVersionMsg(const std_msgs::UInt8& msg);

	int GetJointsNumber();
	uint64_t GetMaskedJoints();
	void SetMaskedJoints(const uint64_t& mask);

	void movementfeedback_callback(const edo_core_msgs::MovementFeedbackConstPtr msg);

	void ackTimeout(State* previous);
	void moveTimeout(State* previous);
};

#endif /* EDO_CORE_PKG_SRC_SUBSCRIBEPUBLISH_H_ */
