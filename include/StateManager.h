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
 * StateManager.h
 *
 *  Created on: Jun 28, 2017
 *      Author: comau
 */

#ifndef EDO_CORE_PKG_SRC_STATEMANAGER_H_
#define EDO_CORE_PKG_SRC_STATEMANAGER_H_

#include "ros/ros.h"

#include "edo_core_msgs/JointStateArray.h"
#include "edo_core_msgs/JointCalibration.h"
#include "edo_core_msgs/JointConfigurationArray.h"
#include "edo_core_msgs/JointReset.h"
#include "edo_core_msgs/MovementCommand.h"
#include "edo_core_msgs/MovementFeedback.h"
#include "edo_core_msgs/JointFwVersionArray.h"
#include "edo_core_msgs/SoftwareVersion.h"
#include "State.h"

class StateManager {
public:
	StateManager();
	virtual ~StateManager();
	void HandleJntState(const edo_core_msgs::JointStateArray& state);
	void HandleJntVersion(const edo_core_msgs::JointFwVersion& msg);
	void HandleReset(const edo_core_msgs::JointReset mask);
	void HandleConfig(const edo_core_msgs::JointConfigurationArray& state);
	void HandleCalibration(const edo_core_msgs::JointCalibration& jointMask);
	void HandleInit(const edo_core_msgs::JointInit msg);
	void HandleJog(const edo_core_msgs::MovementCommand& msg);
	void HandleMove(const edo_core_msgs::MovementCommand& msg);
	void HandleMoveAck(const edo_core_msgs::MovementFeedback& ack);
	void HandleEdoError(const std_msgs::String errorStr);
	void ackTimeout(State* previous);
	void moveTimeout(State* previous);
	void getCurrentState();
	const uint8_t & getMachineState();
	const uint32_t & getMachineOpcode();
	bool getSwVersion(edo_core_msgs::SoftwareVersion::Request &req, edo_core_msgs::SoftwareVersion::Response &res);

	ros::Timer timerJointVersion;
private:

	struct Joint {
		std::string version;
		bool versionReceived;
		uint16_t noReplyCounter;
	};
	void timerJointStateCallback(const ros::TimerEvent& event);
	void timerJointVersionCallback(const ros::TimerEvent& event);
	void setMachineOpcode(const uint8_t bit, const bool set);

	State *current;
	std::vector<Joint> joints;
	std::string usbVersion;
	ros::NodeHandle privateNh;
	ros::Timer timerJointState;
	uint32_t machineOpcode;
};

#endif /* EDO_CORE_PKG_SRC_STATEMANAGER_H_ */
