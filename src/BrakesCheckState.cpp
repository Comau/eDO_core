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
 * BrakeStateCheck.cpp
 *
 *  Created on: May 13, 2019
 *  Author: comau
 */

#include "BrakesCheckState.h"
#include "MoveTestState.h"
#include "EdoMsgType.h"
#include "SubscribePublish.h"

#include <unistd.h>
#include <fcntl.h>

BrakesCheckState* BrakesCheckState::instance = NULL;

BrakesCheckState::BrakesCheckState()
{
  // TODO Auto-generated constructor stub
  machineCurrentState = MACHINE_CURRENT_STATE::BRAKES_CHECK;
}

BrakesCheckState* BrakesCheckState::getInstance()
{
  if (instance == NULL)
  {
    instance = new BrakesCheckState();
  }

  return instance;
}

void BrakesCheckState::getCurrentState()
{
  ROS_INFO("Current State is: BRAKES_CHECK");
}

State* BrakesCheckState::HandleMove(const edo_core_msgs::MovementCommand& msg)
{
  #if 0
  printf ("[CalibrateState,%d] Move Command %c Lin %f\n",__LINE__, msg.move_command, msg.cartesian_linear_speed);
  #endif
  MoveTestState* move = MoveTestState::getInstance();

  // Eseguo il comando...
  return move->StartMove(this, msg);
}

State* BrakesCheckState::HandleMoveAck(const edo_core_msgs::MovementFeedback& ack)
{

  #if 0
  // Invio ack a bridge
  SubscribePublish* SPInstance = SubscribePublish::getInstance();
  if(ack.type != F_NEED_DATA)
    SPInstance->MoveAck(ack);
  return this;
  #endif
  #if 0
  printf ("[CalibrateState,%d] Ack:%d Data:%d\n",__LINE__, ack.type, ack.data);
  #endif
  MoveTestState* move = MoveTestState::getInstance();

  // Eseguo il comando...
  return move->StartAck(this, ack);
}

State* BrakesCheckState::ackCommand()
{
  machineCurrentState = MACHINE_CURRENT_STATE::BRAKES_CHECK;
}