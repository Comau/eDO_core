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
#include <algorithm>

#define NUMBER_KINEMATICS_JOINTS 6
#define NUM_MAX_JOINTS           7

CommandState* CommandState::instance = NULL;

CommandState::CommandState() {
  int jointsNum;
	SPinstance = SubscribePublish::getInstance();
  jointsNum = SPinstance->GetJointsNumber();
  if (jointsNum <= 0)
  {
    jointsNum = NUM_MAX_JOINTS;
  }
	currentState = new InternalState[jointsNum];
	machineCurrentState = MACHINE_CURRENT_STATE::COMMAND_STATE;

	for (int i = 0; i < jointsNum; i++) {
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
	int jointsNumber = SPinstance->GetJointsNumber(); 
	uint32_t jointsMask = (uint32_t)state.joints_mask;

	int commandJoints = 0;
	/* Check if an Ack has been received */
	for (int i = 0; i < jointsNumber; i++, jointsMask >>= 1) {

		if (jointsMask & 1) {
			if ((currentState[i] == WAIT_HIGH) && isHighAck(state.joints[i].commandFlag)) {
//				ROS_INFO("Ack accepted from joint %d", i+1);
				currentState[i] = WAIT_LOW;
			} else if ((currentState[i] == WAIT_LOW) && commandFlagIsLow(state.joints[i].commandFlag)) {
//				ROS_INFO("Ack received from joint %d", i+1);
				currentState[i] = NO_WAIT;
			}
		}

		if(currentState[i] == NO_WAIT) {
			commandJoints++;
		}
	}

	if(commandJoints == jointsNumber) {
		ROS_INFO("Ack command received");
		timerAck.stop();
		return previousState->ackCommand();
	}

	return this;
}

void CommandState::ExecuteCommand(State* state, const edo_core_msgs::JointCalibration& joints) {
	uint32_t jointsMask = (uint32_t)joints.joints_mask;
	int jointsNumber = std::min(SPinstance->GetJointsNumber(), NUMBER_KINEMATICS_JOINTS); 

	for (int i = 0; i < jointsNumber; i++, jointsMask >>= 1) {

		if (jointsMask & 1) {
//			ROS_INFO("CALIB Mask with %d bit set", i);
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
	uint32_t jointsMask = (uint32_t)msg.joints_mask;
	int jointsNumber = std::min(SPinstance->GetJointsNumber(), NUMBER_KINEMATICS_JOINTS); 
  
	for (int i = 0; i < jointsNumber; i++, jointsMask >>= 1) {

		if (jointsMask & 1) {
//			ROS_INFO("RESET Mask with %d bit set", i);
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
	uint32_t jointsMask = (uint32_t)joints.joints_mask;
	int jointsNumber = std::min(SPinstance->GetJointsNumber(), NUMBER_KINEMATICS_JOINTS); 

	for (int i = 0; i < jointsNumber; i++, jointsMask >>= 1) {

		if (jointsMask & 1) {
//			ROS_INFO("CONFIG Mask with %d bit set", i);
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
	uint32_t jointsMask = (uint32_t)msg.joints_mask;
	int jointsNumber = SPinstance->GetJointsNumber(); 

	for (int i = 0; i < jointsNumber; i++, jointsMask >>= 1) {

		if (jointsMask & 1) {
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
  int jointsNumber = SPinstance->GetJointsNumber();
  timerAck.stop();
  if (jointsNumber > 0)
  {
    for (int i = 0; i < jointsNumber; i++)
    {
      if (currentState[i] == 0)
      {
//		    ROS_INFO("        Ack status for joint %d: %d", i+1, currentState[i]);
      }
      else
      {
		    ROS_INFO("Timeout Ack status for joint %d: %d", i+1, currentState[i]);
	  }
    }
  }
  else
  {
		ROS_INFO("No Joints");
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



