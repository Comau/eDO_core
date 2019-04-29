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
 * SubscribePublish.cpp
 *
 *  Created on: Jun 30, 2017
 *      Author: comau
 */

#include "SubscribePublish.h"

SubscribePublish* SubscribePublish::instance = NULL;

SubscribePublish* SubscribePublish::getInstance() {

	if (instance == NULL) {
		instance = new SubscribePublish();
	}

	return instance;
}

SubscribePublish::SubscribePublish()
{
	// -------------- TOPIC FROM/TO BRIDGE NODE --------------
	bridge_jnt_reset_subscriber = node_obj.subscribe("/bridge_jnt_reset",10, &StateManager::HandleReset, &manager);
	bridge_jnt_config_subscriber = node_obj.subscribe("/bridge_jnt_config",10, &StateManager::HandleConfig, &manager);
	bridge_jnt_calib_subscriber = node_obj.subscribe("/bridge_jnt_calib",10, &StateManager::HandleCalibration, &manager);
	bridge_move_subscriber = node_obj.subscribe("/bridge_move",10, &StateManager::HandleMove, &manager);
	bridge_jog_subscriber = node_obj.subscribe("/bridge_jog",10, &StateManager::HandleJog, &manager);
	bridge_init_subscriber = node_obj.subscribe("/bridge_init",10, &StateManager::HandleInit, &manager);
	machine_move_ack_publisher = node_obj.advertise<edo_core_msgs::MovementFeedback>("/machine_movement_ack",10);
	machine_bridge_jnt_state_publisher = node_obj.advertise<edo_core_msgs::JointStateArray>("/machine_bridge_jnt_state",10);
	machine_state_publisher = node_obj.advertise<edo_core_msgs::MachineState>("/machine_state",10);
	bridge_sw_version_server = node_obj.advertiseService("machine_bridge_sw_version_srv", &StateManager::getSwVersion, &manager);

	// -------------- TOPIC FROM/TO ALGORITHM NODE --------------
	algo_jnt_number_client = node_obj.serviceClient<edo_core_msgs::JointsNumber>("/algo_jnt_number_srv");
	algo_move_ack_subscriber = node_obj.subscribe("/algo_movement_ack", 200, &StateManager::HandleMoveAck, &manager);
	
	algo_collision_subscriber = node_obj.subscribe("/algo_collision", 10, &StateManager::HandleAlgoCollision, &manager);							  
	
	machine_algo_jnt_state_publisher = node_obj.advertise<edo_core_msgs::JointStateArray>("/machine_algo_jnt_state", 100);
	machine_move_publisher = node_obj.advertise<edo_core_msgs::MovementCommand>("/machine_move", 100);
	machine_jog_publisher = node_obj.advertise<edo_core_msgs::MovementCommand>("/machine_jog", 100);
	algo_load_configuration_file_client = node_obj.serviceClient<edo_core_msgs::LoadConfigurationFile>("/algo_load_configuration_file_srv");

	// -------------- TOPIC FROM/TO USB/CAN MODULE --------------
	machine_jnt_calib_publisher = node_obj.advertise<edo_core_msgs::JointCalibration>("/machine_jnt_calib",10);
	machine_jnt_reset_publisher = node_obj.advertise<edo_core_msgs::JointReset>("/machine_jnt_reset",10);
	machine_jnt_config_publisher = node_obj.advertise<edo_core_msgs::JointConfigurationArray>("/machine_jnt_config",10);
	machine_init_publisher = node_obj.advertise<edo_core_msgs::JointInit>("/machine_init",10);
	usb_jnt_state_subscriber = node_obj.subscribe("/usb_jnt_state",10, &StateManager::HandleJntState, &manager);
	usb_jnt_version_subscriber = node_obj.subscribe("/usb_jnt_version",10, &StateManager::HandleJntVersion, &manager); //TODO Callback
	machine_jnt_version_publisher = node_obj.advertise<std_msgs::UInt8>("/machine_jnt_version",10);
	
	// -------------- TOPIC FROM/TO RECOVERY NODE ---------------
	recovery_edo_error_subscriber = node_obj.subscribe("/edo_error",10, &StateManager::HandleEdoError, &manager);

	jointsNumber = -1;
	maskedJoints = 0;

	machineStateTimer = node_obj.createTimer(ros::Duration(0.1), &SubscribePublish::timerMachineStateCallback, this);
	machineStateTimer.start();
	
	collTimer = node_obj.createTimer(ros::Duration(11), &SubscribePublish::unbrakeTimerCallback, this, true, false);
}

void SubscribePublish::CalibrationMsg(edo_core_msgs::JointCalibration msg)
{
	ROS_INFO_ONCE("Publish Calibration Msg");
	machine_jnt_calib_publisher.publish(msg);
}

void SubscribePublish::ResetMsg(edo_core_msgs::JointReset msg)
{
	ROS_INFO_ONCE("Publish Reset Msg");
	machine_jnt_reset_publisher.publish(msg);
	//start the collsion timer just after the reset msg is sent
	collTimer.start();
	manager.set_underVoltage_Timer_false();
}

void SubscribePublish::ConfigureMsg(edo_core_msgs::JointConfigurationArray msg)
{
	ROS_INFO_ONCE("Publish Configure Msg");
	machine_jnt_config_publisher.publish(msg);
}

void SubscribePublish::InitMsg(edo_core_msgs::JointInit msg)
{
	ROS_INFO_ONCE("Publish Init Msg");
	machine_init_publisher.publish(msg);
}

void SubscribePublish::JogMsg(const edo_core_msgs::MovementCommand& msg)
{
	machine_jog_publisher.publish(msg);
}

void SubscribePublish::MoveMsg(const edo_core_msgs::MovementCommand& msg)
{
	ROS_INFO_ONCE("Publish Move Msg");
	machine_move_publisher.publish(msg);
}

void SubscribePublish::MoveAck(const edo_core_msgs::MovementFeedback& ack)
{
	// ROS_INFO_ONCE("Publish MoveAck Msg");
	machine_move_ack_publisher.publish(ack);
}

void SubscribePublish::AlgorithmStatusMsg(const edo_core_msgs::JointStateArray& msg)
{
	// ROS_INFO_ONCE("Publish State Msg");
	machine_algo_jnt_state_publisher.publish(msg);
}

void SubscribePublish::BridgeStatusMsg(const edo_core_msgs::JointStateArray& msg)
{
	machine_bridge_jnt_state_publisher.publish(msg);
}

void SubscribePublish::MachineStateMsg(const edo_core_msgs::MachineState& msg)
{
	machine_state_publisher.publish(msg);
}

void SubscribePublish::MachineSwVersionMsg(const std_msgs::UInt8& msg)
{
	manager.timerJointVersion.start();
	machine_jnt_version_publisher.publish(msg);
}

int SubscribePublish::GetJointsNumber() 
{
	edo_core_msgs::JointsNumber srv;

	if (jointsNumber < 0) 
	{
		algo_jnt_number_client.call(srv);

		if(srv.response.counter > 0) 
		{
			jointsNumber = srv.response.counter;
		}
	}
	return jointsNumber;
}

uint64_t SubscribePublish::GetMaskedJoints() {
	return maskedJoints;
}

void SubscribePublish::LoadConfigurationFile() 
{

	edo_core_msgs::LoadConfigurationFile lcf;

  algo_load_configuration_file_client.call(lcf);

  if (lcf.response.result == false)
  {
    ROS_ERROR("Failure loading configuration file");
  }
  return;
}

void SubscribePublish::SetMaskedJoints(const uint64_t& mask) {
	maskedJoints = mask;
}

void SubscribePublish::ackTimeout(State* previous) 
{
	manager.ackTimeout(previous);

}

void SubscribePublish::moveTimeout(State* previous) 
{
	manager.moveTimeout(previous);

}

void SubscribePublish::timerMachineStateCallback(const ros::TimerEvent& event) 
{
	edo_core_msgs::MachineState msg;
	msg.current_state = manager.getMachineState();
	msg.opcode = manager.getMachineOpcode();
	MachineStateMsg(msg);
}

void SubscribePublish::unbrakeTimerCallback(const ros::TimerEvent& event)
{
	//collision timer callback: enables again the collision check that was disabled during the unbrake and recovery

	collTimer.stop();
	
	manager.set_underVoltage_Timer_true();
}