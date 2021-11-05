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
 * BrakeState.cpp
 *
 *  Created on: Jun 30, 2017
 *      Author: comau
 */

#include "BrakeState.h"
#include "CommandState.h"
#include "EdoMsgType.h"
#include "MoveCommandState.h"
#include "NotCalibrateState.h"
#include <cmath>

#include <unistd.h>
#include <fcntl.h>

BrakeState* BrakeState::instance = NULL;

BrakeState::BrakeState() {

  machineCurrentState = MACHINE_CURRENT_STATE::BRAKED;
  SPinstance = SubscribePublish::getInstance();
}

BrakeState* BrakeState::getInstance()
{
  if (instance == NULL) 
  {
    instance = new BrakeState();
  }

  return instance;
}

void BrakeState::getCurrentState()
{
  ROS_INFO("Current State is: BRAKED");
}

State* BrakeState::HandleReset(const edo_core_msgs::JointReset mask)
{
  
  CommandState* command = CommandState::getInstance();

  command->ExecuteCommand(this, mask);
  
  int fd = open("/edo/k3fifo", O_WRONLY);
  if (fd != 0)
  {
    write(fd, "u\n", 2);
    close(fd);
    ROS_INFO("brake state send reset");
  }
  else
  {
    ROS_ERROR("brake state failure opening k3fifo");
  }  
  return command;
}

State* BrakeState::ackCommand()
{
  machineCurrentState = MACHINE_CURRENT_STATE::BRAKED;
  ROS_INFO("brake state rx ack");
  return NotCalibrateState::getInstance();
}

State* BrakeState::HandleMove(const edo_core_msgs::MovementCommand& msg)
{
  #if 0
  printf ("[BrakeState,%d] Move Command %c Lin %f\n",__LINE__, msg.move_command, msg.cartesian_linear_speed);

  if ((msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL) && (msg.cartesian_linear_speed < 0.0f))
  {
    MoveCommandState* move = MoveCommandState::getInstance();

    // Eseguo il comando...
    return move->StartMove(this, msg);
  }
  #endif
  return this;
}