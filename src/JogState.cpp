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
 * JogState.cpp
 *
 *  Created on: Jun 30, 2017
 *      Author: comau
 */

#include "JogState.h"
#include "SubscribePublish.h"

JogState* JogState::instance = NULL;

JogState::JogState() {
	// Duration, callback, calback-owner, oneshot, autostart
	machineCurrentState = MACHINE_CURRENT_STATE::JOG;
	timerMsg = privateNh.createTimer(ros::Duration(0.15), &JogState::timerCallback, this, true, false);
}

void JogState::Init() {
	currentState = STOPPED;
}

JogState* JogState::getInstance() {

	if (instance == NULL) {
		instance = new JogState();
	}

	instance->Init();
	return instance;
}

void JogState::getCurrentState() {

	ROS_INFO("Current State is: JOG");
}

State* JogState::HandleJog(const edo_core_msgs::MovementCommand& msg) {

	SubscribePublish* SPInstance = SubscribePublish::getInstance();

	if ((msg.movement_type == JOG_TRJNT)||(msg.movement_type == JOG_CARLIN)) {

		// Ad ogni refresh di JOG ri-avvio il timer.
		timerMsg.stop();
		
		// Se lo stato è STOPPED è la prima richiesta di JOG...
		if (currentState == STOPPED) {
			currentState = MOVING;
			currentJogCommand = msg;
			SPInstance->JogMsg(msg);
		} else if (currentState == MOVING){
			// Controllo che il vettore dati nel messaggio sia uguale a quello corrente
			if(msg.data != currentJogCommand.data){
				ROS_INFO("Jog command wrong");
				return StopJog();
			}
		}
		
		timerMsg.start();
	}

	return this;
}

State* JogState::ExecuteJog(State* state, const edo_core_msgs::MovementCommand& msg) {
	previousState = state;
	return HandleJog(msg);
}

State* JogState::StopJog() {
	SubscribePublish* SPInstance = SubscribePublish::getInstance();
	edo_core_msgs::MovementCommand msg;
	msg.movement_type = JOG_STOP;
	SPInstance->JogMsg(msg);
	//return HandleJog(msg);
	return previousState;
}

void JogState::timerCallback(const ros::TimerEvent& event) {

	SubscribePublish* SPinstance = SubscribePublish::getInstance();
	timerMsg.stop();
	currentState = STOPPED;
	ROS_INFO("Jog command timed-out");
	SPinstance->moveTimeout(StopJog());
	//StopJog();
}

