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
 * InitState.h
 *
 *  Created on: Jul 12, 2017
 *      Author: comau
 */

#ifndef EDO_CORE_PKG_INCLUDE_INITSTATE_H_
#define EDO_CORE_PKG_INCLUDE_INITSTATE_H_

#include "State.h"
#include "SubscribePublish.h"
#include <tinyxml.h>
#include <ros/package.h>

class InitState: public State {
public:

	static InitState* getInstance();
	void getCurrentState();
	State* HandleInit(const edo_core_msgs::JointInit mask);
	State* ackCommand();
	State* HandleJntVersion(bool timeout);

private:

	enum InternalState {
		WAIT = 0,
		DISCOVERY = 1,
		SET = 2,
		DELETE = 3,
		MASK_JOINTS = 4,
		GET_VERSION = 5,
		GET_VERSION_DONE = 6,
		SET_PID_PARAM = 7,
		JOINTS_RESET = 8
	};

	InitState();
	InitState(InitState const&);
	InitState& operator=(InitState const&);
	void resetState();
	bool loadConfiguration(edo_core_msgs::JointConfigurationArray & msg, const char* pFilename);
	void loadConfiguration( TiXmlNode* pParent, edo_core_msgs::JointConfigurationArray & msg, int & count);
	bool loadAttribute(TiXmlElement* pElement, double & attribute);

	static InitState* instance;
	InternalState currentState;
	SubscribePublish* SPinstance;
	bool* jointInitState;
	unsigned long long int jointToSet;
	bool completeDiscovery;
	uint8_t currentJointVersionID;
	uint8_t versionRetries;
};

#endif /* EDO_CORE_PKG_INCLUDE_INITSTATE_H_ */
