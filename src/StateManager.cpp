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
 * StateManager.cpp
 *
 *  Created on: Jun 28, 2017
 *      Author: comau
 */

#include "ros/ros.h"

#include "StateManager.h"
#include "InitState.h"
#include "NotCalibrateState.h"
#include "SubscribePublish.h"
#include "ErrorState.h"
#include "State.h"
#include "SoftwareVersion.h"
#include "BrakeState.h"

#define JOINT_MAX_UNREPLIES	20

StateManager::StateManager() {

	machineOpcode = 0;
	current = InitState::getInstance();
	ROS_INFO("Start State Manager");
	current->getCurrentState();

	// Duration, callback, calback-owner, oneshot, autostart
	timerJointState = privateNh.createTimer(ros::Duration(3.0), &StateManager::timerJointStateCallback, this, true, false);
	timerJointVersion = privateNh.createTimer(ros::Duration(2.0), &StateManager::timerJointVersionCallback, this, true, false);

}

StateManager::~StateManager() {

	ROS_INFO("Stop State Manager");
}

void StateManager::HandleJntState(const edo_core_msgs::JointStateArray& state) {

	SubscribePublish *SPinstance = SubscribePublish::getInstance();
	bool overCurrent = false;
	bool underVoltage = false;
	bool uncalibrated = false;
	bool jointNoComm = false;
	
	if(state.joints.size() != SPinstance->GetJointsNumber())
		return;
	
	// Inizializzazione
	if(joints.empty()) {
		joints.resize(SPinstance->GetJointsNumber());

		for(Joint j : joints) {
			j.noReplyCounter = 0;
			j.versionReceived = false;
		}
	}
	
	// resetto il timer
	timerJointState.stop();
			
	// controllo la comunicazione su rosserial/usb
	if (getMachineState() != MACHINE_CURRENT_STATE::INIT) {
	
		timerJointState.start();		
	}
	
	for (int i = 0; i < SPinstance->GetJointsNumber(); i++) {
		// controllo se il giunto è presente
		if ((state.joints_mask & (1 << i)) == (0 << i)) {
			// il giunto non ha inviato lo stato
			joints[i].noReplyCounter = joints[i].noReplyCounter + 1;
		} else
			joints[i].noReplyCounter = 0;
		
		if(joints[i].noReplyCounter >= JOINT_MAX_UNREPLIES)
			jointNoComm = true;
	
		// controllo se giunto è in sovracorrente, undervoltage, uncalibrated
		if((state.joints[i].commandFlag & (1 << COMMAND_FLAG::OVERCURRENT)) == (1 << COMMAND_FLAG::OVERCURRENT)){
			overCurrent = true;
			if(current->getMachineCurrentState() != MACHINE_CURRENT_STATE::MACHINE_ERROR)
				current = ErrorState::getInstance();
		}
		
		if((state.joints[i].commandFlag & (1 << COMMAND_FLAG::H_BRIDGE_DOWN)) == (1 << COMMAND_FLAG::H_BRIDGE_DOWN)){
			underVoltage = true;
			if(current->getMachineCurrentState() == MACHINE_CURRENT_STATE::CALIBRATE
				|| current->getMachineCurrentState() == MACHINE_CURRENT_STATE::JOG
				|| current->getMachineCurrentState() == MACHINE_CURRENT_STATE::MOVE
				|| current->getMachineCurrentState() == MACHINE_CURRENT_STATE::NOT_CALIBRATE)
				current = BrakeState::getInstance();
		}
		
		if((state.joints[i].commandFlag & (1 << COMMAND_FLAG::UNCALIBRATED)) == (1 << COMMAND_FLAG::UNCALIBRATED)){
			uncalibrated = true;
			if(current->getMachineCurrentState() == MACHINE_CURRENT_STATE::CALIBRATE){
				current = NotCalibrateState::getInstance();
				current->getCurrentState();
			}
		}
	}
	
	setMachineOpcode(MACHINE_OPCODE::JOINT_OVERCURRENT, overCurrent);
	setMachineOpcode(MACHINE_OPCODE::BRAKE_ACTIVE, underVoltage);
	setMachineOpcode(MACHINE_OPCODE::JOINT_UNCALIBRATED, uncalibrated);
	setMachineOpcode(MACHINE_OPCODE::JOINT_ABSENT, jointNoComm);
	
	// inoltro il messaggio a bridge e algorithms
	SPinstance->BridgeStatusMsg(state);
	
	if(current->getMachineCurrentState() != MACHINE_CURRENT_STATE::BRAKED)
		SPinstance->AlgorithmStatusMsg(state);
	
	// lo stato corrente si occupa di controllare lo stato dei singoli giunti
	current = current->HandleJntState(state);
}

void StateManager::HandleReset(const edo_core_msgs::JointReset mask) {

	current = current->HandleReset(mask);
	current->getCurrentState();
}

void StateManager::HandleCalibration(const edo_core_msgs::JointCalibration& jointMask) {

	current = current->HandleCalibrate(jointMask);
	current->getCurrentState();
}

void StateManager::HandleInit(const edo_core_msgs::JointInit msg) {

	current = current->HandleInit(msg);
	current->getCurrentState();
}

void StateManager::HandleConfig(const edo_core_msgs::JointConfigurationArray& msg) {

	current = current->HandleConfig(msg);
	current->getCurrentState();
}

void StateManager::HandleJog(const edo_core_msgs::MovementCommand& msg) {
	current = current->HandleJog(msg);
	current->getCurrentState();
}

void StateManager::HandleMove(const edo_core_msgs::MovementCommand& msg) {
	current = current->HandleMove(msg);
	current->getCurrentState();
}

void StateManager::HandleMoveAck(const edo_core_msgs::MovementFeedback& ack) {
	current = current->HandleMoveAck(ack);
	current->getCurrentState();
}

void StateManager::ackTimeout(State* previous) {

	ROS_ERROR("Joint Ack timeout");

	if(previous->getMachineCurrentState() == MACHINE_CURRENT_STATE::INIT){
		current = InitState::getInstance();
		current->getCurrentState();
	} else {
		current = ErrorState::getInstance();
		setMachineOpcode(MACHINE_OPCODE::NACK, true);
		current->getCurrentState();
	}
}

void StateManager::moveTimeout(State* previous) {

	ROS_INFO("Move/Jog timed-out");
	current = previous;
	current->getCurrentState();
}

void StateManager::getCurrentState() {

	current->getCurrentState();
}

void StateManager::timerJointStateCallback(const ros::TimerEvent& event) {

	ROS_ERROR("Joints state timed-out");

	current = ErrorState::getInstance();
	setMachineOpcode(MACHINE_OPCODE::JOINT_ABSENT, true);
	current->getCurrentState();
}

void StateManager::timerJointVersionCallback(const ros::TimerEvent& event) {

	ROS_INFO("Joint firmware version timeout");
	timerJointVersion.stop();
	current = current->HandleJntVersion(true);

}

const uint8_t & StateManager::getMachineState(){

	return current->getMachineCurrentState();
}

const uint32_t & StateManager::getMachineOpcode(){

	return machineOpcode;
}

void StateManager::HandleEdoError(const std_msgs::String errorStr) {

	ROS_ERROR_STREAM("EDo Position Error: " << errorStr);

	current = ErrorState::getInstance();
	setMachineOpcode(MACHINE_OPCODE::POSITION_ERROR, true);
}

void StateManager::HandleJntVersion(const edo_core_msgs::JointFwVersion& msg) {
	// software version msg has been received from a joint
	// check which joint is, save the received version and notify the current state

	ROS_INFO("Rx joint version from %d", msg.id);
	timerJointVersion.stop();
	
	if((msg.id > 0) && (msg.id <= joints.size())){
		joints[msg.id - 1].version = "edo_"; 
		joints[msg.id - 1].version += std::to_string(msg.majorRev);
		joints[msg.id - 1].version += ".";
		joints[msg.id - 1].version += std::to_string(msg.minorRev);
		joints[msg.id - 1].version += ".";
		joints[msg.id - 1].version += std::to_string(msg.revision);
		joints[msg.id - 1].version += ".";
		joints[msg.id - 1].version += std::to_string(msg.svn);
		joints[msg.id - 1].versionReceived = true;
	} else if(msg.id == 0){
		// versione del modulo USB
		usbVersion = "edo_"; 
		usbVersion += std::to_string(msg.majorRev);
		usbVersion += ".";
		usbVersion += std::to_string(msg.minorRev);
		usbVersion += ".";
		usbVersion += std::to_string(msg.revision);
		usbVersion += ".";
		usbVersion += std::to_string(msg.svn);
	}

	// notify current state
	current = current->HandleJntVersion(false);
}


bool StateManager::getSwVersion(edo_core_msgs::SoftwareVersion::Request &req, edo_core_msgs::SoftwareVersion::Response &res) {

	SubscribePublish *SPinstance = SubscribePublish::getInstance();
	
	res.version.nodes.resize(SPinstance->GetJointsNumber() + 2); // joints+usb+raspberry
	
	// usb version (id = 0) as first element
	res.version.nodes[0].id = 0;
	res.version.nodes[0].version = usbVersion;
			
	// then insert firmware version of the joints
	for(int i = 0; i < joints.size(); i++){
		res.version.nodes[i+1].id = i+1;
		res.version.nodes[i+1].version = joints[i].version;
	}
		
	// finally, insert software version of Raspberry (id = 255)
	int index = joints.size() + 1;
	
	std::string swVersion;
	swVersion = "edo_"; 
	swVersion += std::to_string(EDO_SW_MAJOR);
	swVersion += ".";
	swVersion += std::to_string(EDO_SW_MINOR);
	swVersion += ".";
	swVersion += std::to_string(EDO_SW_REVISION);
	swVersion += ".";	
	swVersion += std::to_string(EDO_SW_SVN);
	
	printf("Stringa: %s\n", swVersion.c_str());
	res.version.nodes[index].id = 255;
	res.version.nodes[index].version = swVersion;
	
	return true;
}

void StateManager::setMachineOpcode(const uint8_t bit, const bool set){
	
	if(set)
		machineOpcode |= 1 << bit;
	else
		machineOpcode &= ~(1 << bit);
}
