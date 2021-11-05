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
 * MoveCommandState.h
 *
 *  Created on: Jul 3, 2017
 *      Author: comau
 */

#ifndef EDO_CORE_PKG_MOVECOMMANDSTATE_H_
#define EDO_CORE_PKG_MOVECOMMANDSTATE_H_

#include "ros/ros.h"
#include <vector>
#include "State.h"

class MoveCommandState: public State 
{

public:
  static MoveCommandState* getInstance();
  void   getCurrentState();
  State* HandleMove(const edo_core_msgs::MovementCommand& msg);
  void   ExecuteNextMove(const edo_core_msgs::MovementCommand& msg);
  State* HandleMoveAck(const edo_core_msgs::MovementFeedback& ack);
  State* StartMove(State* state, const edo_core_msgs::MovementCommand& msg);
  State* StartAck(State* state, const edo_core_msgs::MovementFeedback& ack);

private:

  MoveCommandState();
  MoveCommandState(MoveCommandState const&);
  MoveCommandState& operator=(MoveCommandState const&);
  void Init();
  bool MessageIsValid(const edo_core_msgs::MovementCommand& msg);
  bool InternalStateIsValid(uint8_t moveType);
  void SendMovementAck(const MESSAGE_FEEDBACK& ackType, int8_t data, int asi_line);

  enum INTERNAL_STATE {
    IDLE   = 0,
    PAUSE  = 1,
    CANCEL = 2,
    RESUME = 3
  };

  static MoveCommandState* instance;
  State* previousState;
  std::vector<edo_core_msgs::MovementCommand> moveMsgBuffer;
  int bufferSize;
  bool firstSent;
  INTERNAL_STATE internalState;
  unsigned long int counterMsgSent;
  unsigned long int counterAckReceived;
};

#endif /* EDO_CORE_PKG_MOVECOMMANDSTATE_H_ */