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
 * NotCalibrateState.cpp
 *
 *  Created on: Jun 30, 2017
 *      Author: comau
 */

#include "NotCalibrateState.h"
#include "CalibrateState.h"
#include "CommandState.h"
#include "JogState.h"
#include "EdoMsgType.h"

#include <cmath>

#include <unistd.h>
#include <fcntl.h>

NotCalibrateState* NotCalibrateState::instance = NULL;

NotCalibrateState::NotCalibrateState() {

	machineCurrentState = MACHINE_CURRENT_STATE::NOT_CALIBRATE;
	SPinstance = SubscribePublish::getInstance();
}

NotCalibrateState* NotCalibrateState::getInstance() {

	if (instance == NULL) {
		instance = new NotCalibrateState();
	}

	return instance;
}

void NotCalibrateState::getCurrentState() {

	ROS_INFO("Current State is: NOT CALIBRATE");
}

State* NotCalibrateState::HandleCalibrate(const edo_core_msgs::JointCalibration& joints) {

	CommandState* command = CommandState::getInstance();

	command->ExecuteCommand(this, joints);

	return command;
}

State* NotCalibrateState::HandleJntState(const edo_core_msgs::JointStateArray& state) {
	
	bool allJointsCalibrated = false;
	
	for (int i = 0; i < SPinstance->GetJointsNumber(); i++) {

		if ((state.joints_mask & (1 << i)) == (1 << i)) {
			if((state.joints[i].commandFlag & (1 << COMMAND_FLAG::UNCALIBRATED)) == 0) {
				allJointsCalibrated = true;
			} else {
				allJointsCalibrated = false;
				break;
			}
		}
	}
	
	if(allJointsCalibrated)
		return CalibrateState::getInstance();
		
	return this;
}

State* NotCalibrateState::HandleJog(const edo_core_msgs::MovementCommand& msg) {

	int jointsOk = 0;

	// Nella fase di calibrazione si accetta solo il movimento di Jog in giunti...
	if ((msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_JOGMOVE) &&
       msg.move_type    == E_MOVE_TYPE::E_MOVE_TYPE_LINEAR) { // Se il movimento è cartesiano...
		ROS_INFO("[%d] Move Command %c not available with MOVE_TYPE %c in current state.",__LINE__,msg.move_command,msg.move_type);
		return this;
	} 
  else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_JOGSTOP) {
		// Se sono in questo stato vuol dire che non ho ancora ricevuto il comando di start
		// altrimenti sarei nello stato JogState
		ROS_INFO("[%d] Jog movement is not active.",__LINE__);
		return this;
	}

	JogState* jog = JogState::getInstance();

	// Eseguo il comando...
	return jog->ExecuteJog(this, msg);
}

State* NotCalibrateState::HandleMoveAck(const edo_core_msgs::MovementFeedback& ack) {

	// Invio ack a bridge
	SubscribePublish* SPInstance = SubscribePublish::getInstance();
	SPInstance->MoveAck(ack);
	return this;
}

State* NotCalibrateState::ackCommand() {

	machineCurrentState = MACHINE_CURRENT_STATE::NOT_CALIBRATE;
}                                                                             
