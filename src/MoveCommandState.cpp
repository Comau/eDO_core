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
 * MoveCommandState.cpp
 *
 *  Created on: Jul 3, 2017
 *      Author: comau
 */

#include "MoveCommandState.h"
#include "SubscribePublish.h"

//#include <unistd.h>
//#include <fcntl.h>

MoveCommandState* MoveCommandState::instance = NULL;

MoveCommandState::MoveCommandState() {
	machineCurrentState = MACHINE_CURRENT_STATE::MOVE;
	bufferSize = 5;
}


void MoveCommandState::Init() {
	moveMsgBuffer.clear();
	firstSent = false;
	internalState = INTERNAL_STATE::IDLE;
	counterMsgSent = 0;
	counterAckReceived = 0;
}

MoveCommandState* MoveCommandState::getInstance() {

	if (instance == NULL) {
		instance = new MoveCommandState();
	}

	instance->Init();
	return instance;
}

void MoveCommandState::getCurrentState() {

	ROS_INFO("Current State is: MOVE");
}

State* MoveCommandState::HandleMoveAck(const edo_core_msgs::MovementFeedback& ack) {

	if (ack.type == COMMAND_EXECUTED) {

		// Aggiorno lo stato interno
		if(internalState == INTERNAL_STATE::CANCEL){
			edo_core_msgs::MovementCommand cancelMsg;
			cancelMsg.move_command = E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL;
			ExecuteNextMove(cancelMsg);
			return previousState;
		} else if(internalState == INTERNAL_STATE::IDLE){ // rimuovo il waypoint eseguito
			if(!moveMsgBuffer.empty()){
				moveMsgBuffer.erase(moveMsgBuffer.begin());
				// std::cout << "Removed point: buf size " << moveMsgBuffer.size() << '\n';
			}
		} else if(internalState == INTERNAL_STATE::RESUME){ // rimuovo il waypoint eseguito
			internalState = INTERNAL_STATE::IDLE;
			// std::cout << "Resume move: buf size " << moveMsgBuffer.size() << '\n';
		}

		counterAckReceived++;
		ROS_INFO("Msg sent %lu", counterMsgSent);
		ROS_INFO("Ack received %lu", counterAckReceived);


		// Inoltro ack a bridge
		SubscribePublish* SPInstance = SubscribePublish::getInstance();
		SPInstance->MoveAck(ack);


		if (internalState == INTERNAL_STATE::IDLE)
		{
			// Invio ad algorithm il nuovo messagggio (se esiste)
			if(!moveMsgBuffer.empty())
				ExecuteNextMove(moveMsgBuffer.front());
			else
				return previousState;

			// Se il buffer era pieno prima di dell'ack, richiedo il waypoint successivo
			if(moveMsgBuffer.size() == bufferSize-1){
				SendMovementAck(F_NEED_DATA, counterMsgSent);
			}
		}

	} else if (ack.type == ERROR) {
		SubscribePublish* SPInstance = SubscribePublish::getInstance();
		SPInstance->MoveAck(ack);
		return previousState;
	} else if (ack.type == COMMAND_RECEIVED || ack.type == F_NEED_DATA) { 
		// non faccio nulla
	} else {
		// se è un ack type diverso da ERROR o COMMAND_EXECUTED e non ho altri punti torno nello stato precedente
		if(moveMsgBuffer.empty())
			return previousState;
	}

	return this;
}

State* MoveCommandState::HandleMove(const edo_core_msgs::MovementCommand& msg) {

	if(!MessageIsValid(msg)){
		SendMovementAck(COMMAND_REJECTED, 0);
		if(moveMsgBuffer.empty())
			return previousState;
		return this;
	}

	if(!InternalStateIsValid(msg.move_command)){
		SendMovementAck(COMMAND_REJECTED, 0);
		if(moveMsgBuffer.empty())
			return previousState;
		return this;
	}

	// messaggio valido, aggiorno lo stato interno
	if ((internalState == INTERNAL_STATE::IDLE) && (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE))
  {
		internalState = INTERNAL_STATE::PAUSE;
	} else if((internalState == INTERNAL_STATE::PAUSE) && (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_RESUME))
  {
		internalState = INTERNAL_STATE::RESUME;
	} else if(msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL)
  {
		internalState = INTERNAL_STATE::CANCEL;
	}

	// controllo il movemente type e genero output
  if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE)
	{
		SendMovementAck(COMMAND_RECEIVED, 0);
		return ExecuteNextMove(msg);
	}
	else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL)
	{
		SendMovementAck(COMMAND_RECEIVED, 0);
		moveMsgBuffer.clear();
		edo_core_msgs::MovementCommand txMsg;
		txMsg.move_command = E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE;
		return ExecuteNextMove(txMsg);
	}
	else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_RESUME)
	{
		SendMovementAck(COMMAND_RECEIVED, 0);
		return ExecuteNextMove(msg);
	}
	else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_MOVE) // e' una move
	{
		if(moveMsgBuffer.size() >= bufferSize){
			SendMovementAck(BUFFER_FULL, 0);
			return this;
		}

		moveMsgBuffer.push_back(msg);

		counterMsgSent++;
		ROS_INFO("Msg sent %lu", counterMsgSent);
		ROS_INFO("Ack received %lu", counterAckReceived);

		SendMovementAck(COMMAND_RECEIVED, int8_t(counterMsgSent));

		// se il buffer non è pieno chiedo il waypoint successivo
		if(moveMsgBuffer.size() < bufferSize)
			SendMovementAck(F_NEED_DATA, int8_t(counterMsgSent));

		// invio il primo waypoint nel buffer se non sono in attesa di un ack
		if(firstSent == false) {
			firstSent = true;
			return ExecuteNextMove(moveMsgBuffer.front());
		}
	}
	else 
	{
	  return nullptr; // Cosa e'?
	}

	return this;
}

State* MoveCommandState::ExecuteNextMove(const edo_core_msgs::MovementCommand& msg) {

	SubscribePublish* SPInstance = SubscribePublish::getInstance();

	// Eseguo il comando
	SPInstance->MoveMsg(msg);
	return this;
}

State* MoveCommandState::StartMove(State* state, const edo_core_msgs::MovementCommand& msg) {

	previousState = state;

	return HandleMove(msg);
}

bool MoveCommandState::MessageIsValid(const edo_core_msgs::MovementCommand& msg) {

  if ((msg.move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_RESUME) &&
      (msg.move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE)  &&
      (msg.move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL) &&
      (msg.move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_MOVE))
  {
    return false;
  }
  
  if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_MOVE)
  {
    if ((msg.move_type != E_MOVE_TYPE::E_MOVE_TYPE_JOINT)    &&
        (msg.move_type != E_MOVE_TYPE::E_MOVE_TYPE_LINEAR)   &&
        (msg.move_type != E_MOVE_TYPE::E_MOVE_TYPE_CIRCULAR))
    {
      return false;
    }
    if ((msg.target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_JOINT)      &&
        (msg.target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION)   &&
        (msg.target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_XTND_POS))
    {
      return false;
    }
  }
  return true;
}

bool MoveCommandState::InternalStateIsValid(uint8_t moveCommand){

  if (internalState == INTERNAL_STATE::CANCEL)
  {
    if((moveCommand != E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL) &&
       (moveCommand != E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE))
      return false;
  }
  else if (internalState == INTERNAL_STATE::PAUSE)
  {
    if((moveCommand != E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL) &&
       (moveCommand != E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE)  &&
       (moveCommand != E_MOVE_COMMAND::E_MOVE_COMMAND_RESUME))
      return false;
  }

  return true;
}

void MoveCommandState::SendMovementAck(const MESSAGE_FEEDBACK& ackType, int8_t data){

	SubscribePublish* SPInstance = SubscribePublish::getInstance();
	edo_core_msgs::MovementFeedback ack;
	ack.type = ackType;
	ack.data = data;
	SPInstance->MoveAck(ack);
}
