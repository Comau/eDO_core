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
#include "CalibrateState.h"
#include "MoveCommandState.h"
#include "BrakesCheckState.h"

#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <string>
#include <dirent.h>

#define ETH_FILE "/sys/class/net/eth0/address"
#define DAT_DIR "/home/edo"
#define SERIAL_PATT "serial_edo_"

#define JOINT_MAX_UNREPLIES	20

#define BC_DELTA_MOVE 	0.05
#define BC_MAX_DEG		10
#define BC_MAX_MOVES 	BC_MAX_DEG/BC_DELTA_MOVE+1
#define BC_BACK_MOVES   1.5/BC_DELTA_MOVE
#define BC_FILE         "/home/edo/state.log"
#define BC_MAX_UNBRAKES 50

#define ENABLE_ROS_INFO  (1==0)
#define ENABLE_COLL_INFO (1==0)
#define ENABLE_BC_INFO   (1==0)

StateManager::StateManager() 
{
	machineOpcode = 0;
	current = InitState::getInstance();
	ROS_INFO("Start State Manager");
	current->getCurrentState();
	
	// Duration, callback, calback-owner, oneshot, autostart
	timerJointState = privateNh.createTimer(ros::Duration(3.0), &StateManager::timerJointStateCallback, this, true, false);
	timerJointVersion = privateNh.createTimer(ros::Duration(2.0), &StateManager::timerJointVersionCallback, this, true, false);
	
	underVoltage_Algo  = false; // collision flag from Algorithm
	
	_tabletFail = false;
	
	bc_flag        = false;
	bc_return_home = false;
	bc_move1 	   = 0;
	bc_move2 	   = 0;
	bc_move3 	   = 0;
	bc_jnt1        = false;
	bc_jnt2        = false;
	bc_jnt3        = false;
	bc_mask        = 0;
	
	brakes_check_start_subscriber = privateNh.subscribe("/brakes_check_start",10, &StateManager::HandleBrakesCheck, this);
	brakes_check_ack_publisher    = privateNh.advertise<edo_core_msgs::BrakesCheckAck>("/brakes_check_ack",10);
}

StateManager::~StateManager() 
{
	ROS_INFO("Stop State Manager");
}


void StateManager::HandleJntState(const edo_core_msgs::JointStateArray& state) 
{
	SubscribePublish *SPinstance = SubscribePublish::getInstance();
	bool overCurrent  = false;
	bool underVoltage = false;
	bool uncalibrated = false;
	bool jointNoComm  = false;
	unsigned long sm_joints_mask;
	int jointsNum = SPinstance->GetJointsNumber();
	
	if((jointsNum <= 0) || (state.joints.size() != jointsNum))
		return;

	// Inizializzazione
	if(joints.empty())
	{
		joints.resize(jointsNum);

		for(Joint j : joints)
		{
			j.noReplyCounter = 0;
			j.versionReceived = false;
		}
	}
    
	// Msg to notify jnt state to websocket
	edo_core_msgs::AppStateArray app_state;
	app_state.joints_mask = state.joints_mask;
	
	// resetto il timer
	timerJointState.stop();

	// controllo la comunicazione su rosserial/usb
	if (getMachineState() != MACHINE_CURRENT_STATE::INIT)
	{
		timerJointState.start();
	}
  
	sm_joints_mask = (unsigned long)state.joints_mask;
	for (int i = 0; i < jointsNum; i++)
	{
		
		app_state.joints.resize(jointsNum);
		
		// controllo se il giunto è presente
		if ((sm_joints_mask & (1 << i)) == 0)
		{
			// il giunto non ha inviato lo stato
			joints[i].noReplyCounter = joints[i].noReplyCounter + 1;
		}
		else
		{
			// Initialization message for the tablet
			
			app_state.joints[i].position = state.joints[i].position;
			joints[i].noReplyCounter = 0;
		}

		if(joints[i].noReplyCounter >= JOINT_MAX_UNREPLIES)
			jointNoComm = true;

		// If there are no errors, continue with the next joint
		if ((state.joints[i].commandFlag & 0xF0) == 0) // COMMAND_FLAG::STATUS_ERROR_MASK) == 0)
			continue;

		// controllo se giunto è in sovracorrente, undervoltage, uncalibrated, rilevata collisione
#if 1
		if((state.joints[i].commandFlag & (1 << COMMAND_FLAG::OVERCURRENT)) != 0)
		{
			overCurrent = true;
		}
#endif
    
		if( (state.joints[i].commandFlag & (1 << COMMAND_FLAG::H_BRIDGE_DOWN)) != 0 && 
		    (current->getMachineCurrentState() != MACHINE_CURRENT_STATE::BRAKED) )
		{
			underVoltage = true;
		}

		if((state.joints[i].commandFlag & (1 << COMMAND_FLAG::UNCALIBRATED)) != 0)
		{
			uncalibrated = true;
		}
	}

	setMachineOpcode(MACHINE_OPCODE::JOINT_OVERCURRENT, overCurrent);
	setMachineOpcode(MACHINE_OPCODE::BRAKE_ACTIVE, underVoltage);
	setMachineOpcode(MACHINE_OPCODE::JOINT_UNCALIBRATED, uncalibrated);
	setMachineOpcode(MACHINE_OPCODE::JOINT_ABSENT, jointNoComm);

#if 1
	// tested in priority order, less priority, first
	if (uncalibrated == true)
	{
		if (current->getMachineCurrentState() == MACHINE_CURRENT_STATE::CALIBRATE)
		{
		  current = NotCalibrateState::getInstance();
		  current->getCurrentState();
		}
	}


	/* Standard collision detection */
	if ((getMachineState() != MACHINE_CURRENT_STATE::BRAKES_CHECK) 	  &&
		(getMachineState() != MACHINE_CURRENT_STATE::MOVE_TEST))
	{

		/* Collision from Algorithm */
		if (underVoltage_Algo == true)
		{
			#if ENABLE_COLL_INFO
			printf("Raspberry\n");
			#endif
			send_BrakeOn();
			setMachineOpcode(MACHINE_OPCODE::BRAKE_ACTIVE, true);
		}
	}
	/* Brakes check */
	else
	{	
		// in brakes check state if a collision is detected then the relative brake is working correctly
		if(!bc_jnt1)
		{
			if(jnt_coll_mask>>0 & 1uL)	// jnt 1 is in collision: the brake is working correctly
			{
				bc_jnt1 = true;
				bc_move1 -= BC_BACK_MOVES;
				if(bc_move1 < 0)
				{
					bc_move1 = 0;
				}
				#if ENABLE_BC_INFO
				printf("Freno 1 OK\n");
				#endif
				bc_mask |= (1uL<<0);	// jnt 1 OK
			}
			if (bc_move1 == BC_MAX_MOVES)
			{
				bc_jnt1 = true;
				bc_move1 = 0;
				#if ENABLE_BC_INFO
				printf("Freno 1 NON funzionante\n");
				#endif
				bc_mask &= ~(1uL<<0);	// jnt 1 NO
			}
		}
		
		if(!bc_jnt2)
		{
			if(jnt_coll_mask>>1 & 1uL)
			{
				bc_jnt2 = true;
				bc_move2 -= BC_BACK_MOVES;
				if(bc_move2 < 0)
				{
					bc_move2 = 0;
				}
				#if ENABLE_BC_INFO
				printf("Freno 2 OK\n");
				#endif
				bc_mask |= (1uL<<1);
			}
			if (bc_move2 == BC_MAX_MOVES)
			{
				bc_jnt2 = true;
				bc_move2 = 0;
				#if ENABLE_BC_INFO
				printf("Freno 2 NON funzionante\n");
				#endif
				bc_mask &= ~(1uL<<1);
			}
		}
		
		if(!bc_jnt3)
		{
			if(jnt_coll_mask>>2 & 1uL)
			{
				bc_jnt3 = true;
				bc_move3 -= BC_BACK_MOVES;
				if(bc_move3 < 0)
				{
					bc_move3 = 0;
				}
				#if ENABLE_BC_INFO
				printf("Freno 3 OK\n");
				#endif
				bc_mask |= (1uL<<2);
			}
			if (bc_move3 == BC_MAX_MOVES)
			{
				bc_jnt3 = true;
				bc_move3 = 0;
				#if ENABLE_BC_INFO
				printf("Freno 3 NON funzionante\n");
				#endif
				bc_mask &= ~(1uL<<2);
			}
		}
	}
  
    /* Collision from Joints */
	if (underVoltage == true)
	{
		#if ENABLE_COLL_INFO
		printf("Joints\n");
		#endif
		send_BrakeOn();
		setMachineOpcode(MACHINE_OPCODE::BRAKE_ACTIVE, true);
	}
		
	/* Tablet disconnection */
	if (_tabletFail)
	{
		#if ENABLE_COLL_INFO
		printf("Tablet\n");
		#endif
		send_BrakeOn();
		_tabletFail = false;
		setMachineOpcode(MACHINE_OPCODE::BRAKE_ACTIVE, true);
	}
  
	if (overCurrent == true)
	{
		if (getMachineState() != MACHINE_CURRENT_STATE::MACHINE_ERROR)
		{
			#if ENABLE_COLL_INFO
			printf("Overcurrent\n");
			#endif
			current = ErrorState::getInstance();
			current->getCurrentState();
		}
	}
#endif
	//
	// inoltro il messaggio a bridge e algorithms
	SPinstance->BridgeStatusMsg(app_state);

	//  if (current->getMachineCurrentState() != MACHINE_CURRENT_STATE::BRAKED)
	SPinstance->AlgorithmStatusMsg(state);

	// lo stato corrente si occupa di controllare lo stato dei singoli giunti
	current = current->HandleJntState(state);
}

void StateManager::HandleReset(const edo_core_msgs::JointReset msg) 
{
	// Controllo file ultimo check
	int   unbrakes_number;
	int   unbrakes_mask;
	int   update_state;
	std::string unbrakes_number_str;
	std::string unbrakes_mask_str;
	std::string update_state_str;
	
	std::ifstream in( BC_FILE );
	
	if( in.is_open() )
	{
		// Read from the State file log
		if(getline(in, unbrakes_number_str))
		{
			if(unbrakes_number_str.empty())
			{
				unbrakes_number = BC_MAX_UNBRAKES;
			}
			else
			{
				sscanf(unbrakes_number_str.c_str(), "%d", &unbrakes_number); 
			}
		}
		else
		{
			unbrakes_number = BC_MAX_UNBRAKES;
		}
	
		if(getline(in, unbrakes_mask_str))
		{
			if(unbrakes_mask_str.empty())
			{
				unbrakes_mask = 7;
			}
			else
			{
				sscanf(unbrakes_mask_str.c_str(), "%d", &unbrakes_mask); 
			}
		}
		else
		{
			unbrakes_mask = 7;
		}
		
		if(getline(in, update_state_str))
		{
			if(update_state_str.empty())
			{
				update_state = 40;
			}
			else
			{
				sscanf(update_state_str.c_str(), "%d", &update_state); 
			}
		}
		else
		{
			update_state = 40;
		}
		
		#if ENABLE_BC_INFO
		printf("Unbrakes : %d\n", unbrakes_number);
		#endif
		#if ENABLE_BC_INFO
		printf("Brakes Mask : %d\n", unbrakes_mask);
		#endif
		#if ENABLE_BC_INFO
		printf("Update State : %d\n", update_state);
		#endif
		
		// Advertise to tablet if a check is necessary or not
		if(unbrakes_number >= BC_MAX_UNBRAKES || unbrakes_number < 0)
		{
			edo_core_msgs::BrakesCheckAck bc_msg;
			bc_msg.state = 3;						// state=3 check necessary
			bc_msg.mask  = uint8_t(unbrakes_mask);
			brakes_check_ack_publisher.publish(bc_msg);
		}			
		
		
		
		// Update the State file with the incremented number of unbrakes
		unbrakes_number++;
	
		std::ofstream out( BC_FILE );
		out << unbrakes_number << std::endl;			
		out << unbrakes_mask << std::endl;
		out << update_state;
	}
	else  
	{
		// Create the file with "standard" values, a check will be necessary
		std::ofstream out( BC_FILE );
		out << BC_MAX_UNBRAKES << std::endl;			
		out << 0 << std::endl;
		out << 40;
	}
	
	current = current->HandleReset(msg);
	current->getCurrentState();
}

void StateManager::HandleCalibration(const edo_core_msgs::JointCalibration& calibMask) 
{
	current = current->HandleCalibrate(calibMask);
	current->getCurrentState();
}

void StateManager::HandleInit(const edo_core_msgs::JointInit msg) 
{
	current = current->HandleInit(msg);
	current->getCurrentState();
}

void StateManager::HandleConfig(const edo_core_msgs::JointConfigurationArray& msg) 
{
	current = current->HandleConfig(msg);
	current->getCurrentState();
}

void StateManager::HandleJog(const edo_core_msgs::MovementCommand& msg) 
{
	current = current->HandleJog(msg);
	current->getCurrentState();
}

void StateManager::HandleMove(const edo_core_msgs::MovementCommand& msg) 
{
	current = current->HandleMove(msg);
	current->getCurrentState();
}

void StateManager::HandleMoveAck(const edo_core_msgs::MovementFeedback& ack) 
{
	current = current->HandleMoveAck(ack);
	current->getCurrentState();
	
	/* Ack handle if in brakes check state */
	if(current->getMachineCurrentState() == MACHINE_CURRENT_STATE::BRAKES_CHECK && ack.type == 2)
	{
		
		if(bc_flag == false)		// the procedure is finished return the mask and go in Calibrate
		{
			#if ENABLE_BC_INFO
			printf("BC Mask: %d\n",bc_mask);
			#endif
			
			edo_core_msgs::BrakesCheckAck bc_msg;
			bc_msg.state = 1;						// state=1 procedure finished
			bc_msg.mask  = bc_mask;
			brakes_check_ack_publisher.publish(bc_msg);
			
			// Update the State log: reset the number of unbrakes and save the new mask
			std::ofstream out( BC_FILE );
			out << 0 << std::endl;			
			out << int(bc_mask) << std::endl;
			out << 40;
			
			// Reset all the variables
			std_msgs::Bool msg_stop;
			msg_stop.data=false;
			HandleBrakesCheck(msg_stop);
			
			current = CalibrateState::getInstance();
			current->getCurrentState();
		}
		else
		{
			if(bc_jnt1 && bc_jnt2 && bc_jnt3)		// every brake has been tested, remove the brakes without the sinusoidal waving and move back in home
			{
				
				if(bc_return_home == false)
				{
					incrementalMoveCreation(bc_move1, bc_move2, bc_move3);
					bc_return_home = true;
				}
				else
				{
					int fd = open("/edo/k3fifo", O_WRONLY);
					if (fd != 0)
					{
					write(fd, "A\n", 2);
					close(fd);
					ROS_INFO("brakes removed");
					}
					else
					{
					ROS_ERROR("brake state failure opening k3fifo");
					}
					ros::Duration(2.0).sleep(); // sleep for two second
				
					incrementalMoveCreation(0, 0, 0);
					bc_flag = false;
				}
			}
			else									// command a new incremental move
			{
				if(!bc_jnt1)
				{
					bc_move1++;
				}
				if(!bc_jnt2)
				{
					bc_move2++;
				}
				if(!bc_jnt3)
				{
					bc_move3++;
				}
				incrementalMoveCreation(bc_move1, bc_move2, bc_move3);
			}
		}
	}
}

void StateManager::HandleAlgoCollision(const edo_core_msgs::CollisionAlgoToState& msg) 
{
	underVoltage_Algo = msg.coll_flag;
	jnt_coll_mask     = msg.joints_mask;
}

void StateManager::ackTimeout(State* previous) 
{
	ROS_ERROR("Joint Ack timeout");
	
	current->getCurrentState();
	
	if(previous->getMachineCurrentState() == MACHINE_CURRENT_STATE::INIT)
	{
		current = InitState::getInstance();
		current->getCurrentState();
	} 
	else 
	{
		current = ErrorState::getInstance();
		setMachineOpcode(MACHINE_OPCODE::NACK, true);
		current->getCurrentState();
	}
}

void StateManager::moveTimeout(State* previous) 
{
	ROS_INFO("Move/Jog timed-out");
	current = previous;
	current->getCurrentState();
}

void StateManager::getCurrentState() 
{
	current->getCurrentState();
}

void StateManager::timerJointStateCallback(const ros::TimerEvent& event) 
{
	ROS_ERROR("Joints state timed-out");

	current = ErrorState::getInstance();
	setMachineOpcode(MACHINE_OPCODE::JOINT_ABSENT, true);
	current->getCurrentState();
}

void StateManager::timerJointVersionCallback(const ros::TimerEvent& event) 
{
	ROS_INFO("Joint firmware version timeout");
	timerJointVersion.stop();
	current = current->HandleJntVersion(true);
}

const uint8_t & StateManager::getMachineState()
{
	return current->getMachineCurrentState();
}

const uint32_t & StateManager::getMachineOpcode()
{
	return machineOpcode;
}

void StateManager::HandleEdoError(const std_msgs::String errorStr) 
{
	ROS_ERROR_STREAM("EDo Position Error: " << errorStr);

	current = ErrorState::getInstance();
	setMachineOpcode(MACHINE_OPCODE::POSITION_ERROR, true);
}

void StateManager::HandleJntVersion(const edo_core_msgs::JointFwVersion& msg) 
{
	// software version msg has been received from a joint
	// check which joint is, save the received version and notify the current state

	ROS_INFO("Rx joint version from %d", msg.id);
	timerJointVersion.stop();
	
	if((msg.id > 0) && (msg.id <= joints.size()))
	{
		joints[msg.id - 1].version = "edo_"; 
		joints[msg.id - 1].version += std::to_string(msg.majorRev);
		joints[msg.id - 1].version += ".";
		joints[msg.id - 1].version += std::to_string(msg.minorRev);
		joints[msg.id - 1].version += ".";
		joints[msg.id - 1].version += std::to_string(msg.revision);
		joints[msg.id - 1].version += ".";
		joints[msg.id - 1].version += std::to_string(msg.svn);
		joints[msg.id - 1].versionReceived = true;
	} 
	else if(msg.id == 0)
	{
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


bool StateManager::getInfo(std::string& hwinfo, std::string& swinfo) 
{
	std::ifstream infile;
  
	// read HW info (mac address)
	infile.open(ETH_FILE);
  
	if(infile.is_open())
	{  
		if(std::getline(infile, hwinfo))
		{
			infile.close();
		}
		
		// read SW info	(license file)
		DIR *dir;
		struct dirent *ent;

		if ((dir = opendir (DAT_DIR)) != NULL) 
		{
			/* search all the files and directories within directory */
			while ((ent = readdir (dir)) != NULL) 
			{		
				std::string file_name = ent->d_name;
				std::string file_pattern = SERIAL_PATT;

				std::string::size_type found = file_name.find(file_pattern);	// return -1 if no matches are found, 
																				// return  0 if a match is found from the first character
				if(found == 0)
				{	
					infile.open(ent->d_name);
					if(infile.is_open())
					{
						if(std::getline(infile, swinfo))
						{
							infile.close();
							ROS_INFO("LICENSE FILE READ: %s, [%s]",file_name.c_str(), swinfo.c_str());
							return true;
						}
					}
					else
					{
						ROS_ERROR("Error in opening the license file");
					}
				}	
			}
			closedir (dir);
		} 
		else 
		{
			ROS_ERROR("Error in opening the license directory");
		}	
	}
	else
	{
		ROS_ERROR("Error in reading the mac address");	
	}

	return false;
}


bool StateManager::getSwVersion(edo_core_msgs::SoftwareVersion::Request &req, edo_core_msgs::SoftwareVersion::Response &res) {
  int numjoints = 0;
  int index = 0;
  std::string swVersion;
  std::string hwInfo("mac");
  std::string swInfo("no license");
  SubscribePublish *SPinstance = SubscribePublish::getInstance();

  // The version can be called before the Initialization
  numjoints = SPinstance->GetJointsNumber();

  swVersion = "edo_"; 
  swVersion += std::to_string(EDO_SW_MAJOR);
  swVersion += ".";
  swVersion += std::to_string(EDO_SW_MINOR);
  swVersion += ".";
  swVersion += std::to_string(EDO_SW_REVISION);
  swVersion += ".";
  swVersion += std::to_string(EDO_SW_SVN);
  
  if (numjoints > 0 ) 
  {
    res.version.nodes.resize(numjoints + 4); // joints+usb+raspberry + mac + key

    // usb version (id = 0) as first element
    res.version.nodes[index].id = 0;
    res.version.nodes[index].version = usbVersion;

    // insert software version of Raspberry (id = 255)
    res.version.nodes[++index].id = 255;
    res.version.nodes[index].version = swVersion;
    
    getInfo(hwInfo,swInfo);
    
    res.version.nodes[++index].id = 253;
    res.version.nodes[index].version = hwInfo;
    
    res.version.nodes[++index].id = 254;
    res.version.nodes[index].version = swInfo;
    
    // then insert firmware version of the joints
    for(int i = index; i < joints.size(); i++)
    {
      res.version.nodes[i+1].id = i+1;
      res.version.nodes[i+1].version = joints[i].version;
    }
 
  }
  else
  {
    res.version.nodes.resize(4); // raspberry only + mac + key
    index = 0;
    
    // usb version (id = 0) as first element
    res.version.nodes[index].id = 0;
    res.version.nodes[index].version = "usb";
    // insert software version of Raspberry (id = 255)
    
    res.version.nodes[++index].id = 255;
    res.version.nodes[index].version = swVersion;
    
    getInfo(hwInfo,swInfo);
    
    res.version.nodes[++index].id = 253;
    res.version.nodes[index].version = hwInfo;
    
    res.version.nodes[++index].id = 254;
    res.version.nodes[index].version = swInfo;
  }
  
  return true;
}

void StateManager::setMachineOpcode(const uint8_t bit, const bool set)
{
	if(set)
		machineOpcode |= 1 << bit;
	else
		machineOpcode &= ~(1 << bit);
}

void StateManager::HB_fail_Callback(std_msgs::Bool msg)
{
	_tabletFail = msg.data;
}

void StateManager::send_BrakeOn()
{
	/* collision or emergency stop during brakes check */
	if ((current->getMachineCurrentState() == MACHINE_CURRENT_STATE::BRAKES_CHECK) 	  ||
		(current->getMachineCurrentState() == MACHINE_CURRENT_STATE::MOVE_TEST))
		{
			edo_core_msgs::BrakesCheckAck bc_msg;
			bc_msg.state = 2;						// state=2 procedure failed
			bc_msg.mask = 0;
			brakes_check_ack_publisher.publish(bc_msg);
			
			#if ENABLE_BC_INFO
			printf("BC procedure interrupted: mask %d\n",bc_mask);
			#endif
			
			std_msgs::Bool msg_stop;
			msg_stop.data=false;
			HandleBrakesCheck(msg_stop);
		}
			
			
	if ((current->getMachineCurrentState() == MACHINE_CURRENT_STATE::CALIBRATE) 	  ||
		(current->getMachineCurrentState() == MACHINE_CURRENT_STATE::JOG)             ||
		(current->getMachineCurrentState() == MACHINE_CURRENT_STATE::MOVE)            ||
		(current->getMachineCurrentState() == MACHINE_CURRENT_STATE::MOVE_TEST)       ||
		(current->getMachineCurrentState() == MACHINE_CURRENT_STATE::BRAKES_CHECK)    ||
		(current->getMachineCurrentState() == MACHINE_CURRENT_STATE::NOT_CALIBRATE))
		{
	
			current = BrakeState::getInstance();
			current->getCurrentState();

			int fd = open("/edo/k3fifo", O_WRONLY);
			if (fd != 0)
			{
				write(fd, "o\n", 2);
				close(fd);
				ROS_INFO("send brake on");
			}
			else
			{
				ROS_ERROR("brake on state failure opening k3fifo");
			}
		}
}

// Brakes check callback
void StateManager::HandleBrakesCheck(std_msgs::Bool msg)
{
	if(msg.data)
	{
		// Msgs to set double check thresholds
		SubscribePublish *SPinstance = SubscribePublish::getInstance();
		
		edo_core_msgs::JointInit thr_jnt;
		thr_jnt.mode = 3;
		thr_jnt.joints_mask = 64;
		thr_jnt.reduction_factor = 10.0;
		SPinstance->machine_init_publisher.publish(thr_jnt);
		
		edo_core_msgs::CollisionThreshold thr_rasp;
		thr_rasp.joints_mask = 64;
		thr_rasp.threshold = 10.0;
		SPinstance->algo_coll_thr_publisher.publish(thr_rasp);
		
		// Ack to the tablet that the procedure has started
		edo_core_msgs::BrakesCheckAck bc_msg;
		bc_msg.state = 0;						// state=0 procedure started
		bc_msg.mask = 0;
		brakes_check_ack_publisher.publish(bc_msg);
		
		#if ENABLE_BC_INFO
		printf("Ack to tablet for BC\n");
		#endif
		
		bc_flag   = true;
		bc_return_home = false;
		bc_move1  = 0;
		bc_move2  = 0;
		bc_move3  = 0;
		bc_jnt1   = false;
		bc_jnt2   = false;
		bc_jnt3   = false;
		bc_mask   = 0;
		
		//Writes a 'B' in the k3fifo, special char for brakes check procedure: send brakes on but keeps the motors on

		current = BrakesCheckState::getInstance();
		current->getCurrentState();
		
		int fd = open("/edo/k3fifo", O_WRONLY);
		if (fd != 0)
		{
			write(fd, "B\n", 2);
			close(fd);
			ROS_INFO("send brake on for check");
		}
		else
		{
			ROS_ERROR("brake on state failure opening k3fifo");
			return;
		}

		ros::Duration(2.0).sleep(); // sleep for two seconds
		
		incrementalMoveCreation(0, 0, 0);
	}
	else
	{
		// Msgs to set double check thresholds
		SubscribePublish *SPinstance = SubscribePublish::getInstance();
		
		edo_core_msgs::JointInit thr_jnt;
		thr_jnt.mode = 3;
		thr_jnt.joints_mask = 64;
		thr_jnt.reduction_factor = 0.3;
		SPinstance->machine_init_publisher.publish(thr_jnt);
		
		edo_core_msgs::CollisionThreshold thr_rasp;
		thr_rasp.joints_mask = 64;
		
		thr_rasp.threshold = 0.3;
		SPinstance->algo_coll_thr_publisher.publish(thr_rasp);
		
		bc_flag   = false;
		bc_return_home = false;
		bc_move1  = 0;
		bc_move2  = 0;
		bc_move3  = 0;
		bc_jnt1   = false;
		bc_jnt2   = false;
		bc_jnt3   = false;
		bc_mask   = 0;
	}
}

// Function to create the small incremental moves for the brakes check
void StateManager::incrementalMoveCreation(int move1, int move2, int move3)
{
	#if ENABLE_BC_INFO
	printf("%.2f %.2f %.2f\n",move1*BC_DELTA_MOVE,move2*BC_DELTA_MOVE,move3*BC_DELTA_MOVE);
	#endif
	
	edo_core_msgs::MovementCommand move_msg;
	
	move_msg.move_command = 77;
	move_msg.move_type = 74;
	move_msg.ovr = 30;
	move_msg.delay = 0;
	move_msg.remote_tool = 0;
	move_msg.cartesian_linear_speed = 0.0;
	
	edo_core_msgs::Point target_point;
	target_point.data_type        = 74;
	target_point.cartesian_data.x = 0;
	target_point.cartesian_data.y = 0;
	target_point.cartesian_data.z = 0;
	target_point.cartesian_data.a = 0;
	target_point.cartesian_data.e = 0;
	target_point.cartesian_data.r = 0;
	target_point.cartesian_data.config_flags = "";
	target_point.joints_mask      = 127;
	target_point.joints_data      = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	target_point.joints_data[0] = BC_DELTA_MOVE * move1;
	target_point.joints_data[1] = BC_DELTA_MOVE * move2;
	target_point.joints_data[2] = BC_DELTA_MOVE * move3;
	move_msg.target = target_point;
			
	edo_core_msgs::Point via_point;
	target_point.data_type        = 0;
	target_point.cartesian_data.x = 0;
	target_point.cartesian_data.y = 0;
	target_point.cartesian_data.z = 0;
	target_point.cartesian_data.a = 0;
	target_point.cartesian_data.e = 0;
	target_point.cartesian_data.r = 0;
	target_point.cartesian_data.config_flags = "";
	target_point.joints_mask    = 0;
	target_point.joints_data    = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	move_msg.via = via_point;
	
	edo_core_msgs::Frame zero_frame;
	zero_frame.x = 0;
	zero_frame.y = 0;
	zero_frame.z = 0;
	zero_frame.a = 0;
	zero_frame.e = 0;
	zero_frame.r = 0;
	move_msg.tool  = zero_frame;
	move_msg.frame = zero_frame;
	
	HandleMove(move_msg);
}