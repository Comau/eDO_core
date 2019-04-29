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

#include <unistd.h>
#include <fcntl.h>

#include <fstream>
#include <string>
#include <dirent.h>

#define ETH_FILE "/sys/class/net/eth0/address"
#define DAT_DIR "/home/edo"
#define SERIAL_PATT "serial_edo_"

#define JOINT_MAX_UNREPLIES	20

StateManager::StateManager() 
{
	machineOpcode = 0;
	current = InitState::getInstance();
	ROS_INFO("Start State Manager");
	current->getCurrentState();

	// Duration, callback, calback-owner, oneshot, autostart
	timerJointState = privateNh.createTimer(ros::Duration(3.0), &StateManager::timerJointStateCallback, this, true, false);
	timerJointVersion = privateNh.createTimer(ros::Duration(2.0), &StateManager::timerJointVersionCallback, this, true, false);
	
	underVoltage_Algo = false; // collision flag from Algorithm
	underVoltage_Timer = true; // if true the collision check is enabled, if false no because we are in unbrake and recovery phase
}

StateManager::~StateManager() 
{
	ROS_INFO("Stop State Manager");
}


void StateManager::HandleJntState(const edo_core_msgs::JointStateArray& state) 
{
	SubscribePublish *SPinstance = SubscribePublish::getInstance();
	bool overCurrent = false;
	bool underVoltage = false;

	bool uncalibrated = false;
	bool jointNoComm = false;
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
    // controllo se il giunto è presente
    if ((sm_joints_mask & (1 << i)) == 0)
    {
      // il giunto non ha inviato lo stato
      joints[i].noReplyCounter = joints[i].noReplyCounter + 1;
    }
    else
    {
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
#if 0
      if(current->getMachineCurrentState() != MACHINE_CURRENT_STATE::MACHINE_ERROR)
      {
        current = ErrorState::getInstance();
        current->getCurrentState();
      }
#endif
    }
#endif
    
    if((state.joints[i].commandFlag & (1 << COMMAND_FLAG::H_BRIDGE_DOWN)) != 0)
    {
      underVoltage = true;
#if 0
      if((current->getMachineCurrentState() == MACHINE_CURRENT_STATE::CALIBRATE) ||
         (current->getMachineCurrentState() == MACHINE_CURRENT_STATE::JOG)       ||
         (current->getMachineCurrentState() == MACHINE_CURRENT_STATE::MOVE)      ||
         (current->getMachineCurrentState() == MACHINE_CURRENT_STATE::NOT_CALIBRATE))
      {
        current = BrakeState::getInstance();
        current->getCurrentState();
        {
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
#endif
    }

    if((state.joints[i].commandFlag & (1 << COMMAND_FLAG::UNCALIBRATED)) != 0)
    {
      uncalibrated = true;
#if 0
      if(current->getMachineCurrentState() == MACHINE_CURRENT_STATE::CALIBRATE)
      {
        current = NotCalibrateState::getInstance();
        current->getCurrentState();
      }
#endif
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
  if (underVoltage == true)
  {
    if ((current->getMachineCurrentState() == MACHINE_CURRENT_STATE::CALIBRATE) ||
        (current->getMachineCurrentState() == MACHINE_CURRENT_STATE::JOG)       ||
        (current->getMachineCurrentState() == MACHINE_CURRENT_STATE::MOVE)      ||
        (current->getMachineCurrentState() == MACHINE_CURRENT_STATE::NOT_CALIBRATE))
    {
      current = BrakeState::getInstance();
      current->getCurrentState();
      {
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
  }
	/* Collision from Algorithm */

	if (underVoltage_Algo == true && underVoltage_Timer == true)
	{

		if ((current->getMachineCurrentState() == MACHINE_CURRENT_STATE::CALIBRATE) ||
			(current->getMachineCurrentState() == MACHINE_CURRENT_STATE::JOG)       ||
			(current->getMachineCurrentState() == MACHINE_CURRENT_STATE::MOVE)      ||
			(current->getMachineCurrentState() == MACHINE_CURRENT_STATE::NOT_CALIBRATE))
		{
			current = BrakeState::getInstance();
			current->getCurrentState();
			{
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
		//underVoltage_Algo = false;
	}
  if (overCurrent == true)
  {
    if (current->getMachineCurrentState() != MACHINE_CURRENT_STATE::MACHINE_ERROR)
    {
      current = ErrorState::getInstance();
      current->getCurrentState();
    }
  }
#endif

  // inoltro il messaggio a bridge e algorithms
  SPinstance->BridgeStatusMsg(state);

//  if (current->getMachineCurrentState() != MACHINE_CURRENT_STATE::BRAKED)
    SPinstance->AlgorithmStatusMsg(state);

  // lo stato corrente si occupa di controllare lo stato dei singoli giunti
  current = current->HandleJntState(state);
}

void StateManager::HandleReset(const edo_core_msgs::JointReset mask) 
{
	current = current->HandleReset(mask);
	current->getCurrentState();
}

void StateManager::HandleCalibration(const edo_core_msgs::JointCalibration& jointMask) 
{
	current = current->HandleCalibrate(jointMask);
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
}

void StateManager::HandleAlgoCollision(const edo_core_msgs::CollisionFromAlgoToState& msg) 
{
	if (msg.coll_flag == true){
		underVoltage_Algo = true;
	}
	else{
		underVoltage_Algo = false;
	}
}

void StateManager::ackTimeout(State* previous) 
{
	ROS_ERROR("Joint Ack timeout");

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


bool StateManager::getInfo(std::string& hwinfo, std::string& swinfo) {

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

void StateManager::set_underVoltage_Timer_true()
{
		underVoltage_Timer=true;
}

void StateManager::set_underVoltage_Timer_false()
{
		underVoltage_Timer=false;
}