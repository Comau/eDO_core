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
 * State.h
 *
 *  Created on: Jun 30, 2017
 *      Author: comau
 */

#ifndef STATE_H_
#define STATE_H_

#include "std_msgs/String.h"
#include "edo_core_msgs/JointStateArray.h"
#include "edo_core_msgs/JointCalibration.h"
#include "edo_core_msgs/JointReset.h"
#include "edo_core_msgs/JointInit.h"
#include "edo_core_msgs/JointConfigurationArray.h"
#include "edo_core_msgs/MovementCommand.h"
#include "edo_core_msgs/MovementFeedback.h"
#include "edo_core_msgs/JointFwVersion.h"
#include "edo_core_msgs/NodeSwVersionArray.h"
#include "ros/ros.h"
#include "EdoMsgType.h"

enum MACHINE_CURRENT_STATE {
	COMMAND_STATE = 255, /* Stato temporaneno quando c'è un comando in esecuzione */
	INIT = 0, /* Stato iniziale */
	NOT_CALIBRATE = 1, /* macchina non calibrata */
	CALIBRATE = 2, /* macchina calibrata */
	MOVE = 3, /* macchina in esecuzione di una move */
	JOG = 4, /* macchina in esecuzione di un jog */
	MACHINE_ERROR = 5, /* macchina in stato di error e in attesa  di un riavvio */
	BRAKED = 6 /* brake active, no motor power supply */
};

enum MACHINE_OPCODE {
	NACK = 0, /* Bit 0 - Ack non ricevuto/i da almeno un giunto */
	JOINT_ABSENT = 1, /* Bit 1 - Un giunto non sta pubblicando il proprio stato. Ferma tutto. */
	JOINT_OVERCURRENT = 2, /* Bit 2 - Allarme di saturazione di corrente su un giunto. Ferma tutto. */
	JOINT_UNCALIBRATED = 3, /* Bit 3 - Allarme di uncalibrated su un giunto. Solo il jog giunti è accettato. */
	POSITION_ERROR = 4, /* Bit 4 - Errore di posizione della macchina. Ferma tutto. */
	ROSSERIAL_ERROR = 5, /* Bit 5 - Errore su rosserial, no stato dei giunti. Ferma tutto. */
	BRAKE_ACTIVE = 6, /* Bit 6 - Freno inserito, no potenza motori */
	EMERGENCY_STOP = 7, /* Bit 7 - Fungo attivo */
	FENCE = 8 /* Bit - Fence attivo */
};

enum COMMAND_FLAG {
	IDLE = 0x00,
	ACK_INIT = 0x01, /* ack a un messaggio di init */
	ACK_CALIBRATION = 0x02, /* ack a un messaggio di calibrazione */
	ACK_CONFIG = 0x03, /* ack a un messaggio di configurazione pid */
	ACK_RESET = 0x04, /* ack a un messaggio di reset */
	ACK_FW_VERSION = 0x05, /* ack a un messaggio di firmware version */
	H_BRIDGE_DOWN = 5, /* bit 5 - ponte h aperto - no potenza motori */
	OVERCURRENT = 6, /* bit 6 -sovracorrente */
	UNCALIBRATED = 7, /* bit 7 - giunto non calibrato */
};
	
class State {
public:
	State();
	virtual ~State();
	virtual void getCurrentState() = 0;
	virtual State* HandleJntState(const edo_core_msgs::JointStateArray& state);
	virtual State* HandleCalibrate(const edo_core_msgs::JointCalibration& joints);
	virtual State* HandleReset(const edo_core_msgs::JointReset mask);
	virtual State* HandleConfig(const edo_core_msgs::JointConfigurationArray& joints);
	virtual State* HandleInit(const edo_core_msgs::JointInit mask);
	virtual State* HandleMove(const edo_core_msgs::MovementCommand& msg);
	virtual State* HandleJog(const edo_core_msgs::MovementCommand& msg);
	virtual State* HandleMoveAck(const edo_core_msgs::MovementFeedback& ack);
	virtual State* ackCommand();
	virtual State* HandleJntVersion(bool timeout);

	const uint8_t & getMachineCurrentState();

protected:

	uint8_t machineCurrentState;
	uint32_t machineOpcode;

};

#endif /* STATE_H_ */
