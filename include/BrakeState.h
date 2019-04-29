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
 * BrakeState.h
 *
 *  Created on: Jun 30, 2017
 *      Author: comau
 */

#ifndef EDO_CORE_PKG_SRC_BRAKESTATE_H_
#define EDO_CORE_PKG_SRC_BRAKESTATE_H_

#include "std_msgs/String.h"

#include "State.h"
#include "SubscribePublish.h"

class BrakeState: public State {
public:
	static BrakeState* getInstance();
	void getCurrentState();
	State* HandleMove(const edo_core_msgs::MovementCommand& msg);
	State* HandleReset(const edo_core_msgs::JointReset mask);
	State* ackCommand();

private:


	BrakeState();
	BrakeState(BrakeState const&);
	BrakeState& operator=(BrakeState const&);

	static BrakeState* instance;
	SubscribePublish* SPinstance;
};

#endif /* EDO_CORE_PKG_SRC_BRAKESTATE_H_ */
