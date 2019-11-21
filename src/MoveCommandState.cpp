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
 *  Author: comau
 */

#include "MoveCommandState.h"
#include "SubscribePublish.h"

#if STATE_MACHINE_DEVELOPMENT_RELEASE
#define ENABLE_MINIMAL_MCS_PRINTFS (1==0)
#define ENABLE_ROS_INFO  (1==0)
#else
#define ENABLE_MINIMAL_MCS_PRINTFS (1==0)
#define ENABLE_ROS_INFO  (1==0)
#endif

MoveCommandState* MoveCommandState::instance = NULL;

MoveCommandState::MoveCommandState() {
  machineCurrentState = MACHINE_CURRENT_STATE::MOVE;
  bufferSize = 5;
}


void MoveCommandState::Init() {
  moveMsgBuffer.clear();
  if (moveMsgBuffer.capacity() < bufferSize)
  {
    moveMsgBuffer.reserve(bufferSize);
  }
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
#if ENABLE_ROS_INFO
  ROS_INFO("Current State is: MOVE");
#endif
  return;
}

#if ENABLE_MINIMAL_MCS_PRINTFS
static const char *sam_AckTypes[6] = {
  "BUFFER_FULL     (-3)",
  "COMMAND_REJECTED(-2)", /* comando scartato */
  "ERROR           (-1)", /* l’esecuzione della move è stata interrotta per un errore */
  "COMMAND_RECEIVED( 0)", /* acknowledgement, comando ricevuto */
  "F_NEED_DATA     ( 1)", /* si chiede il punto successivo */
  "COMMAND_EXECUTED( 2)"  /* movimento completato */
};

static void printAck(const edo_core_msgs::MovementFeedback& ack, const char *apc_func, int asi_line)
{ 
  printf("printAckType [%s,%d] <%s> Data %d\n",apc_func,asi_line,sam_AckTypes[ack.type-MESSAGE_FEEDBACK::BUFFER_FULL],ack.data);  
  return;
}
#endif

State* MoveCommandState::HandleMoveAck(const edo_core_msgs::MovementFeedback& ack) {

#if ENABLE_MINIMAL_MCS_PRINTFS
  printAck(ack,"MoveCommandState::HandleMoveAck from Algo",__LINE__);
#endif
  if (ack.type == COMMAND_EXECUTED) 
  {
    // Aggiorno lo stato interno
    if (internalState == INTERNAL_STATE::CANCEL)
    {
#if ENABLE_MINIMAL_MCS_PRINTFS
printf ("[HandleMoveAck,%d] Force a CANCEL Move Command Size:%d\n",__LINE__,moveMsgBuffer.size());
#endif
      edo_core_msgs::MovementCommand cancelMsg;
      cancelMsg.move_command = E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL;
      ExecuteNextMove(cancelMsg);
      return previousState;
    }
    else if (internalState == INTERNAL_STATE::RESUME)
    {
      internalState = INTERNAL_STATE::IDLE;
    }

    // Inoltro ack a bridge
#if ENABLE_MINIMAL_MCS_PRINTFS
printf ("[HandleMoveAck,%d] Data %d to Bridge\n",__LINE__,ack.data);
#endif
    SubscribePublish* SPInstance = SubscribePublish::getInstance();
    SPInstance->MoveAck(ack);

    if (internalState == INTERNAL_STATE::IDLE)
    {
      // Invio ad algorithm il nuovo messagggio (se esiste)
      if ( (moveMsgBuffer.empty()) && (_algorithm_state != ALGORITHM_STATE::MOVING) )
        return previousState;
	  else if ( (moveMsgBuffer.empty()) && (_algorithm_state == ALGORITHM_STATE::MOVING) )
		return this;
	
      ExecuteNextMove(moveMsgBuffer.front());
#if ENABLE_MINIMAL_MCS_PRINTFS
printf ("[MoveCommandState,%d] ENM MCS:%d send to ALGO\n",__LINE__,moveMsgBuffer.size());
#endif
      moveMsgBuffer.erase(moveMsgBuffer.begin()); // e lo rimuovo dalla coda
    }
  }
  else if (ack.type == F_NEED_DATA) 
  {
    if (internalState == INTERNAL_STATE::IDLE)
    {
      // Inoltro ack a bridge
#if ENABLE_MINIMAL_MCS_PRINTFS
      printAck(ack,"[HandleMoveAck,%d] to Bridge F_NEED_DATA",__LINE__);
#endif
      SubscribePublish* SPInstance = SubscribePublish::getInstance();
      SPInstance->MoveAck(ack);

      // Invio ad algorithm il MOVE Command successivo(se la UI lo ha gia' inviato)
      // Se ho un buffer in coda, lo invio
      if (!moveMsgBuffer.empty())
      {
        ExecuteNextMove(moveMsgBuffer.front());
#if ENABLE_MINIMAL_MCS_PRINTFS
        printf ("[MoveCommandState,%d] ENM in FLY MCS:%d send to ALGO\n",__LINE__,moveMsgBuffer.size());
#endif
        moveMsgBuffer.erase(moveMsgBuffer.begin()); // e lo rimuovo dalla coda
        return this;
      }
#if ENABLE_MINIMAL_MCS_PRINTFS
      else
      {
        printf ("[MoveCommandState,%d] No Move Commands available Size:%d\n",__LINE__,moveMsgBuffer.size());
      }
#endif
    }
  } 
  else if ((ack.type == ERROR) || (ack.type == BUFFER_FULL) || (ack.type == COMMAND_REJECTED))
  {
    SubscribePublish* SPInstance = SubscribePublish::getInstance();
    SPInstance->MoveAck(ack);
    return previousState;
  } 
#if ENABLE_MINIMAL_MCS_PRINTFS
  else if (ack.type == COMMAND_RECEIVED)
  { 
    // non faccio nulla
    printAck(ack,"MoveCommandState::HandleMoveAck non faccio nulla",__LINE__);
  }
#endif
  else
  {
    // se è un ack type diverso e non ho altri punti torno nello stato precedente
    if (moveMsgBuffer.empty())
      return previousState;
  }

  return this;
}

State* MoveCommandState::HandleMove(const edo_core_msgs::MovementCommand& msg) {

  if (!MessageIsValid(msg))
  {
#if ENABLE_MINIMAL_MCS_PRINTFS
printf ("[HandleMove,%d] Internal state %d Move Command %c Rejected\n",__LINE__, internalState, msg.move_command);
#endif
    SendMovementAck(COMMAND_REJECTED, 0, __LINE__);
    return this;
  }

  if (!InternalStateIsValid(msg.move_command))
  {
#if ENABLE_MINIMAL_MCS_PRINTFS
printf ("[HandleMove,%d] Internal state %d Move Command %c Rejected\n",__LINE__, internalState, msg.move_command);
#endif
    if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_MOVE) // e' una move
    {
      // It is possible to rx a MOVE in PAUSE state due to some timing issues.
      // It is not a big issue. Just put it into the queue and wait for a better time
      moveMsgBuffer.push_back(msg);
      SendMovementAck(COMMAND_RECEIVED, 0, __LINE__);
    }
    else
    {      
      SendMovementAck(COMMAND_REJECTED, 0, __LINE__);
    }
    return this;
  }

  // messaggio valido

  // controllo il Move Command type e genero output
  if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE)
  {
    SendMovementAck(COMMAND_RECEIVED, 0, __LINE__);
    // Aggiorno lo stato interno
    if (internalState == INTERNAL_STATE::IDLE)
    {
      internalState = INTERNAL_STATE::PAUSE;
      // Send the PAUSE command to Algorithm Manager
#if ENABLE_MINIMAL_MCS_PRINTFS
printf ("[HandleMove,%d] Internal state %d Move Command %c send to ALGO\n",__LINE__, internalState, msg.move_command);
#endif
      ExecuteNextMove(msg);
      return this;
    }
#if ENABLE_MINIMAL_MCS_PRINTFS
    else
    {
printf ("[HandleMove,%d] Internal state %d Move Command %c\n",__LINE__, internalState, msg.move_command);
    }
#endif
  }
  else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL)
  {
    SendMovementAck(COMMAND_RECEIVED, 0, __LINE__);
    // Aggiorno lo stato interno
    internalState = INTERNAL_STATE::CANCEL;
    // Filtering the CANCEL command for Algorithm Manager
    // First empty the buffer
    while (!moveMsgBuffer.empty()) 
    {
#if ENABLE_MINIMAL_MCS_PRINTFS
      printf ("[HandleMove,%d] flush MCS:%d\n",__LINE__,moveMsgBuffer.size());
#endif
      moveMsgBuffer.erase(moveMsgBuffer.begin()); // e lo rimuovo dalla coda
      SendMovementAck(COMMAND_EXECUTED, 20, __LINE__);
    }
    // moveMsgBuffer.clear();
    // Create another MOVE Command with a PAUSE request.
    edo_core_msgs::MovementCommand txMsg;
    txMsg.move_command = E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE;
    // Send the PAUSE command to Algorithm Manager
    ExecuteNextMove(txMsg);
#if ENABLE_MINIMAL_MCS_PRINTFS
printf ("[HandleMove,%d] Internal state %d Move Command %c but send to ALGO a PAUSE request\n",__LINE__, internalState, msg.move_command);
#endif
    return this;
  }
  else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_RESUME)
  {
    SendMovementAck(COMMAND_RECEIVED, 0, __LINE__);
     if (internalState == INTERNAL_STATE::PAUSE)
    {
      // Aggiorno lo stato interno
      internalState = INTERNAL_STATE::IDLE;
#if ENABLE_MINIMAL_MCS_PRINTFS
printf ("[HandleMove,%d] Internal state %d Move Command %c send to ALGO\n",__LINE__, internalState, msg.move_command);
#endif
      // Send the RESUME command to Algorithm Manager
      ExecuteNextMove(msg);
      return this;
    }
#if ENABLE_MINIMAL_MCS_PRINTFS
    else
    {
printf ("[HandleMove,%d] Internal state %d Move Command %c\n",__LINE__, internalState, msg.move_command);
    }
#endif
  }
  else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_MOVE) // e' una move
  {
    if(moveMsgBuffer.size() >= bufferSize)
    {
      SendMovementAck(COMMAND_RECEIVED, 0, __LINE__);    
      SendMovementAck(COMMAND_EXECUTED, 100, __LINE__);
      SendMovementAck(BUFFER_FULL, 0, __LINE__);
      return this;
    }

    // invio il waypoint ad Algo
     if (internalState == INTERNAL_STATE::IDLE)
    {
      if (!moveMsgBuffer.empty()) 
      {
        moveMsgBuffer.push_back(msg);
        ExecuteNextMove(moveMsgBuffer.front());
#if ENABLE_MINIMAL_MCS_PRINTFS
        printf ("[MoveCommandState,%d] ENM in FLY MCS:%d send to ALGO\n",__LINE__,moveMsgBuffer.size());
#endif
        moveMsgBuffer.erase(moveMsgBuffer.begin()); // e lo rimuovo dalla coda
      }
      else
      {
        ExecuteNextMove(msg);
      }
#if ENABLE_MINIMAL_MCS_PRINTFS
      printf ("[HandleMove,%d] ENM MCS:%d sent to ALGO\n",__LINE__,moveMsgBuffer.size());
#endif
    }
    else
    {
      moveMsgBuffer.push_back(msg);
#if ENABLE_MINIMAL_MCS_PRINTFS
      printf ("[HandleMove,%d] RX Cmd Move MCS:%d\n",__LINE__,moveMsgBuffer.size());
#endif
    }
    SendMovementAck(COMMAND_RECEIVED, 0, __LINE__);    
  }
  else 
  {
#if ENABLE_MINIMAL_MCS_PRINTFS
printf ("[HandleMove,%d] Internal state %d Move Command %c Cosa e'?\n",__LINE__, internalState, msg.move_command);
#endif
    return previousState;  // Cosa e'?
  }

  return this;
}

void MoveCommandState::ExecuteNextMove(const edo_core_msgs::MovementCommand& msg) {

	SubscribePublish* SPInstance = SubscribePublish::getInstance();
	
	// Eseguo il comando
	SPInstance->MoveMsg(msg);
	return;
}

State* MoveCommandState::StartMove(State* state, const edo_core_msgs::MovementCommand& msg) {

  previousState = state;
  return HandleMove(msg);
}

State* MoveCommandState::StartAck(State* state, const edo_core_msgs::MovementFeedback& ack) {

  previousState = state;

  return HandleMoveAck(ack);
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

void MoveCommandState::SendMovementAck(const MESSAGE_FEEDBACK& ackType, int8_t data, int asi_line){

  SubscribePublish* SPInstance = SubscribePublish::getInstance();
  edo_core_msgs::MovementFeedback ack;
  ack.type = ackType;
  ack.data = data;
#if ENABLE_MINIMAL_MCS_PRINTFS
  printAck(ack,"Send Ack To Bridge",asi_line);
#endif

  SPInstance->MoveAck(ack);
}