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
 * CommandState.cpp
 *
 *  Created on: Jul 4, 2017
 *      Author: comau
 */

#include "CommandState.h"

CommandState* CommandState::instance = NULL;

CommandState::CommandState() {

	SPinstance = SubscribePublish::getInstance();
	currentState = new InternalState[SPinstance->GetJointsNumber()];
	machineCurrentState = MACHINE_CURRENT_STATE::COMMAND_STATE;

	for (int i = 0; i < SPinstance->GetJointsNumber(); i++) {
		currentState[i] = NO_WAIT;
	}

	previousState = nullptr;

	// Duration, callback, calback-owner, oneshot, autostart
	timerAck = privateNh.createTimer(ros::Duration(10), &CommandState::timerCallback, this, true, false);
}

CommandState* CommandState::getInstance() {

	if (instance == NULL) {
		instance = new CommandState();
	}
	
	return instance;
}

void CommandState::getCurrentState() {

	ROS_INFO("Current State is: COMMAND");
}

State* CommandState::HandleJntState(const edo_core_msgs::JointStateArray& state) {

	SubscribePublish* SPinstance = SubscribePublish::getInstance();

	int commandJoints = 0;

	/* Check if an Ack has been received */
	for (int i = 0; i < SPinstance->GetJointsNumber(); i++) {

		if ((state.joints_mask & (1 << i)) == (1 << i)) {
			if ((currentState[i] == WAIT_HIGH) && isHighAck(state.joints[i].commandFlag)) {
				currentState[i] = WAIT_LOW;
			} else if ((currentState[i] == WAIT_LOW) && commandFlagIsLow(state.joints[i].commandFlag)) {
				ROS_INFO("Ack received for joint %d", i+1);
				currentState[i] = NO_WAIT;
			}
		}

		if(currentState[i] == NO_WAIT) {
			commandJoints++;
		}
	}

	if(commandJoints == SPinstance->GetJointsNumber()) {
		ROS_INFO("Ack command received");
		timerAck.stop();
		return previousState->ackCommand();
	}

	return this;
}

void CommandState::ExecuteCommand(State* state, const edo_core_msgs::JointCalibration& joints) {

	for (int i = 0; i < SPinstance->GetJointsNumber(); i++) {

		if ((joints.joints_mask & (1 << i)) == (1 << i)) {
			ROS_INFO("CALIB Mask with %d bit setted", i);
			currentState[i] = WAIT_HIGH;
		} else {
			currentState[i] = NO_WAIT;
		}
	}

	previousState = state;

	timerAck.start();
	SPinstance->CalibrationMsg(joints);
}

void CommandState::ExecuteCommand(State* state, const edo_core_msgs::JointReset msg) {

	for (int i = 0; i < SPinstance->GetJointsNumber(); i++) {

		if ((msg.joints_mask & (1 << i)) == (1 << i)) {
			ROS_INFO("RESET Mask with %d bit setted", i);
			currentState[i] = WAIT_HIGH;
		} else {
			currentState[i] = NO_WAIT;
		}
	}

	previousState = state;

	timerAck.start();
	SPinstance->ResetMsg(msg);
}

void CommandState::ExecuteCommand(State* state, const edo_core_msgs::JointConfigurationArray& joints) {

	for (int i = 0; i < SPinstance->GetJointsNumber(); i++) {

		if ((joints.joints_mask & (1 << i)) == (1 << i)) {
			ROS_INFO("CONFIG Mask with %d bit setted", i);
			currentState[i] = WAIT_HIGH;
		} else {
			currentState[i] = NO_WAIT;
		}
	}

	previousState = state;

	timerAck.start();
	SPinstance->ConfigureMsg(joints);
}

void CommandState::ExecuteCommand(State* state, const edo_core_msgs::JointInit msg) {

	for (int i = 0; i < SPinstance->GetJointsNumber(); i++) {

		if ((msg.joints_mask & (1 << i)) == (1 << i)) {
			currentState[i] = WAIT_HIGH;
		} else {
			currentState[i] = NO_WAIT;
		}
	}

	previousState = state;

	timerAck.start();
	SPinstance->InitMsg(msg);
}

void CommandState::ExecuteCommand(State* state, const std_msgs::UInt8 msg) {

	previousState = state;

	timerAck.start();
	SPinstance->MachineSwVersionMsg(msg);
}

void CommandState::timerCallback(const ros::TimerEvent& event) {

	timerAck.stop();
	for (int i = 0; i < SPinstance->GetJointsNumber(); i++) {
		ROS_INFO("Ack status for joint %d: %d", i+1, currentState[i]);
	}
	SPinstance->ackTimeout(previousState);
}

bool CommandState::isHighAck(const uint8_t & commandFlag) {

	if((commandFlag & 0x0F) == COMMAND_FLAG::ACK_INIT
		|| (commandFlag & 0x0F) == COMMAND_FLAG::ACK_CALIBRATION
		|| (commandFlag & 0x0F) == COMMAND_FLAG::ACK_RESET
		|| (commandFlag & 0x0F) == COMMAND_FLAG::ACK_CONFIG
		|| (commandFlag & 0x0F) == COMMAND_FLAG::ACK_FW_VERSION) {
		
		return true;
	}
		
	return false;
}

bool CommandState::commandFlagIsLow(const uint8_t & commandFlag) {

	if((commandFlag & 0x0F) == COMMAND_FLAG::IDLE) {
		
		return true;
	}
		
	return false;
}



