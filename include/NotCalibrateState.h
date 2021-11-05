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
 * NotCalibrateState.h
 *
 *  Created on: Jun 30, 2017
 *      Author: comau
 */

#ifndef EDO_CORE_PKG_SRC_NOTCALIBRATESTATE_H_
#define EDO_CORE_PKG_SRC_NOTCALIBRATESTATE_H_

#include "std_msgs/String.h"

#include "State.h"
#include "SubscribePublish.h"

class NotCalibrateState: public State
{

public:
  static NotCalibrateState* getInstance();
  void getCurrentState();
  State* HandleJntState(const edo_core_msgs::JointStateArray& state);
  State* HandleCalibrate(const edo_core_msgs::JointCalibration& joints);
  State* HandleJog(const edo_core_msgs::MovementCommand& msg);
  State* HandleMoveAck(const edo_core_msgs::MovementFeedback& ack);
  State* ackCommand();

private:

  enum InternalJointState {
    NO_STATE_RECEIVED = 0,
    NO_CALIBRATE = 1,
    TO_BE_CALIBRATE = 2,
    CALIBRATE = 3
  };

  NotCalibrateState();
  NotCalibrateState(NotCalibrateState const&);
  NotCalibrateState& operator=(NotCalibrateState const&);
  State* CalibrateJoints();

  static NotCalibrateState* instance;
  InternalJointState* jointCalibrationState;
  SubscribePublish* SPinstance;
};

#endif /* EDO_CORE_PKG_SRC_NOTCALIBRATESTATE_H_ */