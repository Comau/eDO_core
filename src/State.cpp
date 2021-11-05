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
 * State.cpp
 *
 *  Created on: Jun 30, 2017
 *      Author: comau
 */

#include "State.h"
#include "SubscribePublish.h"

State::State()
{
  // TODO Auto-generated constructor stub
  //machineCurrentState = MACHINE_CURRENT_STATE::INIT;
  //machineOpcode = 0;
  ros::Subscriber algorithm_state_subscriber = node_obj.subscribe("/algorithm_state",10, &State::AlgorithmStateCallback, this);
}

State::~State()
{
  // TODO Auto-generated destructor stub
}

State* State::HandleJntState(const edo_core_msgs::JointStateArray& state)
{
  return this;
}

State* State::HandleCalibrate(const edo_core_msgs::JointCalibration& joints)
{
  ROS_INFO("Calibrate Command not available in current state");
  getCurrentState();
  return this;
}

State* State::HandleReset(const edo_core_msgs::JointReset mask)
{
  ROS_INFO("Reset Command not available in current state");
  getCurrentState();
  return this;
}

State* State::HandleConfig(const edo_core_msgs::JointConfigurationArray& joints)
{
  ROS_INFO("Configure Command not available in current state");
  getCurrentState();
  return this;
}

State* State::HandleInit(const edo_core_msgs::JointInit mask)
{
  ROS_INFO("Init Command not available in current state");
  getCurrentState();
  return this;
}

State* State::HandleMove(const edo_core_msgs::MovementCommand& msg)
{
  ROS_INFO("Move Command not available in current state");
  getCurrentState();
  return this;
}

State* State::HandleMoveAck(const edo_core_msgs::MovementFeedback& ack)
{
  ROS_INFO("Received a MovementFeedback in a NOT Movement State");
  getCurrentState();
  return this;
}

State* State::HandleJog(const edo_core_msgs::MovementCommand& msg)
{
  ROS_INFO("Jog Command not available in current state");
  getCurrentState();
  return this;
}

State* State::ackCommand()
{
  return this;
}

State* State::HandleJntVersion(bool timeout)
{
  return this;
}

const uint8_t & State::getMachineCurrentState()
{
  return machineCurrentState;
}

void State::AlgorithmStateCallback(const std_msgs::Int8 msg)
{
  _algorithm_state = msg.data;
}