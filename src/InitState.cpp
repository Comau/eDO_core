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
 * InitState.cpp
 *
 *  Created on: Jul 12, 2017
 *      Author: comau
 */
#include "InitState.h"
#include "CommonService.h"
#include "CommandState.h"
#include "NotCalibrateState.h"
#include <tinyxml.h>
#include <math.h>

#define HANDLE_VERSION 1
#define VERSION_RETRIES 3
#define DEVELOPMENT_RELEASE (1==0)


bool handle_config = true;
bool handle_brake_disengage = false;

InitState* InitState::instance = NULL;

InitState::InitState() {

	jointInitState = nullptr;
}

void InitState::resetState() {

	if(jointInitState != nullptr) {
		delete[] jointInitState;
	}

	jointInitState = nullptr;
	jointToSet = 0;
	completeDiscovery = false;
	currentJointVersionID = 0;
	machineCurrentState = MACHINE_CURRENT_STATE::INIT;
	currentState = WAIT;
	versionRetries = 0;
}

InitState* InitState::getInstance() {

	if (instance == NULL) {
		instance = new InitState();
	}

	instance->resetState();
	return instance;
}

void InitState::getCurrentState() {
	ROS_INFO("Current State is: INIT");
	ros::param::get("~sendPid", handle_config);
	ROS_INFO("Send PID Params: %s", ((handle_config) ? "True" : "False"));
	ros::param::get("~sendReset", handle_brake_disengage);
	ROS_INFO("Send Brake Disengage : %s", ((handle_brake_disengage) ? "True" : "False"));
}

State* InitState::HandleInit(const edo_core_msgs::JointInit msg) {
  std::string pkg_path_ = ros::package::getPath("edo_core_pkg");
  FILE *fptr;
  unsigned long      sm_joints_mask;
  int                jointsNum;

  fptr = fopen((pkg_path_ + "/config/" + "LoadCnfgFileReport.log").c_str(),"a");
  if (fptr == NULL)
    fptr = stdout;

  /* /home/edo/edo_core_catkin_ws/src/edo_core_pkg */
  fprintf(fptr, "-------------------------------------------------------------\n");
 
  sm_joints_mask = (unsigned long)msg.joints_mask;
  fprintf(fptr, "[%s,%d] Mode: %d Mask: %d\n",__FILE__,__LINE__, msg.mode, sm_joints_mask);
   // Se stiamo facendo la discovery di tutti i giunti del sistema e si riceve
  // l'ack di questo messaggio, vuol dire che tutti i giunti hanno risposto
  // quindi il sistema è inizializzato e si può uscire dallo stato di init
  if (msg.mode == 0) {
    currentState = DISCOVERY;
    unsigned int jointsCount = CommonService::countBitMask(sm_joints_mask);

    fprintf(fptr, "[%s,%d] Requested a robot of: %d axes\n",__FILE__,__LINE__,jointsCount);
    fprintf(fptr, "[%s,%d] %s\n",__FILE__,__LINE__,("rm "+ pkg_path_ + "/config/"+"robot.edo").c_str());
    std::system(("rm "+pkg_path_ + "/config/"+"robot.edo").c_str());
    if (jointsCount==4)
    { /* 4 axes e.DO */
      fprintf(fptr, "[%s,%d] %s\n",__FILE__,__LINE__,("/bin/cp "+pkg_path_ + "/config/"+"robot4.edo"+" "+pkg_path_ + "/config/"+"robot.edo").c_str());
      std::system(("/bin/cp "+pkg_path_ + "/config/"+"robot4.edo"+" "+pkg_path_ + "/config/"+"robot.edo").c_str());
    }
    else if (jointsCount==7)
    {
      fprintf(fptr, "[%s,%d] %s\n",__FILE__,__LINE__,("/bin/cp "+pkg_path_ + "/config/"+"robot7.edo"+" "+pkg_path_ + "/config/"+"robot.edo").c_str());
      std::system(("/bin/cp "+pkg_path_ + "/config/"+"robot7.edo"+" "+pkg_path_ + "/config/"+"robot.edo").c_str());
    }
    else
    { 
      if(msg.reduction_factor != 0.0)
      {
         /* Default is 6 axes e.DO */
        fprintf(fptr, "[%s,%d] %s\n",__FILE__,__LINE__,("/bin/cp "+pkg_path_ + "/config/"+"robot6_1.edo"+" "+pkg_path_ + "/config/"+"robot.edo").c_str());
        std::system(("/bin/cp "+pkg_path_ + "/config/"+"robot6_1.edo"+" "+pkg_path_ + "/config/"+"robot.edo").c_str());
      }
      else
      {
        /* Default is 6 axes e.DO */
        fprintf(fptr, "[%s,%d] %s\n",__FILE__,__LINE__,("/bin/cp "+pkg_path_ + "/config/"+"robot6.edo"+" "+pkg_path_ + "/config/"+"robot.edo").c_str());
        std::system(("/bin/cp "+pkg_path_ + "/config/"+"robot6_0.edo"+" "+pkg_path_ + "/config/"+"robot.edo").c_str());
      }
    }
    {
      SPinstance = SubscribePublish::getInstance();
      SPinstance->LoadConfigurationFile();

      jointsNum = SPinstance->GetJointsNumber();
    }
    fprintf(fptr, "[%s,%d] Initialized a robot of: %d axes\n",__FILE__,__LINE__, jointsNum);
      
    if (fptr != stdout)
      fclose(fptr);

    if(jointsNum > 0)
    {
      if (jointInitState == nullptr)
      {
        jointInitState = new bool[jointsNum];

        for (int i = 0; i < jointsNum; i++) 
        {
          jointInitState[i] = false;
        }
      }

      ROS_INFO("Requested   a robot of: %d axes", jointsCount);
      ROS_INFO("Initialized a robot of: %d axes", jointsNum);
      if (jointsCount == jointsNum)
      {
        ROS_INFO("Complete Discovery");
        completeDiscovery = true;
      }
    }
  }
  else if (msg.mode == 1)
  {
    SPinstance = SubscribePublish::getInstance();
    currentState = SET;
    jointsNum = SPinstance->GetJointsNumber();
    // Salvo quale giunto sta per essere settato
    for (int i = 0; i < jointsNum; i++) {
      if ((msg.joints_mask & (1 << i)) == msg.joints_mask) {
        jointToSet = i;
        break;
      }
    }
  }
  else if (msg.mode == 2)
  {
    currentState = DELETE;
  }
  else if (msg.mode == 4)
  {
    SPinstance = SubscribePublish::getInstance();
    currentState = MASK_JOINTS;
    SPinstance->SetMaskedJoints(msg.joints_mask);
    return this;
  }

  {
    CommandState* commandInstance = CommandState::getInstance();
    commandInstance->ExecuteCommand(this, msg);

    return commandInstance;
  }
}

State* InitState::ackCommand() {

	machineCurrentState = MACHINE_CURRENT_STATE::INIT;

	if ((currentState == DISCOVERY) && (completeDiscovery == true))
	{
#if HANDLE_VERSION
		currentState = GET_VERSION;
		currentJointVersionID = 0;
		return HandleJntVersion(true);
#else
		return NotCalibrateState::getInstance();
#endif

	} else if (currentState == SET) {

		jointInitState[jointToSet] = true;
		int initializedJoints = 0;
		int jointsNum = SPinstance->GetJointsNumber();

		for (int i = 0; i < jointsNum; i++) {

			if(jointInitState[i] == true) {
				initializedJoints++;
			}
		}

		if (initializedJoints == jointsNum) {

			return NotCalibrateState::getInstance();
		}
	} else if (currentState == GET_VERSION_DONE) {
    
		if(handle_config){
			currentState = SET_PID_PARAM;
			edo_core_msgs::JointConfigurationArray msg;
			msg.joints_mask = pow(2, SPinstance->GetJointsNumber()) - 1;
			std::string filepath = ros::package::getPath("edo_core_pkg") + "/config/" + "pid.xml";
			if(!loadConfiguration(msg, filepath.c_str())){
				ROS_INFO("Error loading PID params from xml");
				return this;
			}
			CommandState* commandInstance = CommandState::getInstance();
			commandInstance->ExecuteCommand(this, msg);
			return commandInstance;
		} else if(handle_brake_disengage){ 
			ROS_INFO("Sending brake disengage command");
			currentState = JOINTS_RESET;
			edo_core_msgs::JointReset msg;
			msg.joints_mask = pow(2, SPinstance->GetJointsNumber()) - 1;
			CommandState* commandInstance = CommandState::getInstance();
			commandInstance->ExecuteCommand(this, msg);
			return commandInstance;
		} else		
			return NotCalibrateState::getInstance();
		
	} else if (currentState == SET_PID_PARAM) {	

		if(handle_brake_disengage){
			ROS_INFO("Sending brake disengage command");
			currentState = JOINTS_RESET;
			edo_core_msgs::JointReset msg;
			msg.joints_mask = pow(2, SPinstance->GetJointsNumber()) - 1;
			CommandState* command = CommandState::getInstance();
			command->ExecuteCommand(this, msg);
			return command;
		} else
			return NotCalibrateState::getInstance();
	} else if (currentState == JOINTS_RESET)
		return NotCalibrateState::getInstance();

	return this;
}

State* InitState::HandleJntVersion(bool timeout)
{
	if(timeout && ++versionRetries <= VERSION_RETRIES) {
		ROS_INFO("Request Version Info to %d", currentJointVersionID);
		std_msgs::UInt8 msg;
		msg.data = currentJointVersionID;
		CommandState* commandInstance = CommandState::getInstance();
		commandInstance->ExecuteCommand(this, msg);
		return commandInstance;
	} else { // received a firmware version msg or max retries reached

		currentJointVersionID++;
		versionRetries = 0;
    if (currentJointVersionID == 1)
    {
			ROS_INFO("Version Info received");
			currentState = GET_VERSION_DONE;
			return ackCommand();
    }
    else if(currentJointVersionID <= SPinstance->GetJointsNumber()){
			ROS_INFO("Request Version Info to %d", currentJointVersionID);
			std_msgs::UInt8 msg;
			msg.data = currentJointVersionID;
			CommandState* commandInstance = CommandState::getInstance();
			commandInstance->ExecuteCommand(this, msg);
			return commandInstance;
		} else {
			ROS_INFO("Version Info received");
			currentState = GET_VERSION_DONE;
			return ackCommand();
		}

	}
}

bool InitState::loadAttribute(TiXmlElement* pElement, double & attribute)
{
	if ( !pElement ) return 0;

	TiXmlAttribute* pAttrib = pElement->FirstAttribute();

	while (pAttrib)
	{
		if(strcmp(pAttrib->Name(), "value") == 0){
		
			if (pAttrib->QueryDoubleValue(&attribute)==TIXML_SUCCESS){
				return true;
			} else {
				return false;
			}
		}
		
		pAttrib=pAttrib->Next();
	}
	return false;
}

void InitState::loadConfiguration( TiXmlNode* pParent, edo_core_msgs::JointConfigurationArray & msg, int & countJoints)
{
	if ( !pParent ) return;

	TiXmlNode* pChild;
	TiXmlText* pText;
	int t = pParent->Type();

	switch ( t )
	{
	case TiXmlNode::TINYXML_DOCUMENT:
		//printf( "Document" );
		break;

	case TiXmlNode::TINYXML_ELEMENT:
		//printf( "Element: [%s]", pParent->Value());
		if(strcmp(pParent->Value(), "Joint") == 0){
			countJoints++;
//			printf(" XML parse; joint counter: %d\n", countJoints);
		} else if(strcmp(pParent->Value(), "Kpp") == 0)
		{
			double kpp;
			loadAttribute(pParent->ToElement(), kpp);
			msg.joints[countJoints-1].kp = kpp;
		} else if(strcmp(pParent->Value(), "Tpi") == 0)
		{
			double tpi;
			loadAttribute(pParent->ToElement(), tpi);
			msg.joints[countJoints-1].ti = tpi;
		} else if(strcmp(pParent->Value(), "Tpd") == 0)
		{
			double tpd;
			loadAttribute(pParent->ToElement(), tpd);
			msg.joints[countJoints-1].td = tpd;
		} else if(strcmp(pParent->Value(), "Satp") == 0)
		{
			double satp;
			loadAttribute(pParent->ToElement(), satp);
			msg.joints[countJoints-1].sat = satp;
		} else if(strcmp(pParent->Value(), "Maxp") == 0)
		{
			double maxp;
			loadAttribute(pParent->ToElement(), maxp);
			msg.joints[countJoints-1].max = maxp;
		} else if(strcmp(pParent->Value(), "Kffp") == 0)
		{
			double kffp;
			loadAttribute(pParent->ToElement(), kffp);
			msg.joints[countJoints-1].kff = kffp;
		} else if(strcmp(pParent->Value(), "Kvp") == 0)
		{
			double kvp;
			loadAttribute(pParent->ToElement(), kvp);
			msg.joints[countJoints-1].kpv = kvp;
		} else if(strcmp(pParent->Value(), "Tvi") == 0)
		{
			double tvi;
			loadAttribute(pParent->ToElement(), tvi);
			msg.joints[countJoints-1].tiv = tvi;
		} else if(strcmp(pParent->Value(), "Tvd") == 0)
		{
			double tvd;
			loadAttribute(pParent->ToElement(), tvd);
			msg.joints[countJoints-1].tdv = tvd;
		} else if(strcmp(pParent->Value(), "Satv") == 0)
		{
			double satv;
			loadAttribute(pParent->ToElement(), satv);
			msg.joints[countJoints-1].satv = satv;
		} else if(strcmp(pParent->Value(), "Maxv") == 0)
		{
			double maxv;
			loadAttribute(pParent->ToElement(), maxv);
			msg.joints[countJoints-1].maxv = maxv;
		} else if(strcmp(pParent->Value(), "Kffv") == 0)
		{
			double kffv;
			loadAttribute(pParent->ToElement(), kffv);
			msg.joints[countJoints-1].kffv = kffv;
		} else if(strcmp(pParent->Value(), "Kcp") == 0)
		{
			double kpt;
			loadAttribute(pParent->ToElement(), kpt);
			msg.joints[countJoints-1].kpt = kpt;
		} else if(strcmp(pParent->Value(), "Tci") == 0)
		{
			double tit;
			loadAttribute(pParent->ToElement(), tit);
			msg.joints[countJoints-1].tit = tit;
		} else if(strcmp(pParent->Value(), "Tcd") == 0)
		{
			double tdt;
			loadAttribute(pParent->ToElement(), tdt);
			msg.joints[countJoints-1].tdt = tdt;
		} else if(strcmp(pParent->Value(), "Satc") == 0)
		{
			double satt;
			loadAttribute(pParent->ToElement(), satt);
			msg.joints[countJoints-1].satt = satt;
		} else if(strcmp(pParent->Value(), "Kffc") == 0)
		{
			double kfft;
			loadAttribute(pParent->ToElement(), kfft);
			msg.joints[countJoints-1].kfft = kfft;
		} else if(strcmp(pParent->Value(), "Maxc") == 0)
		{
			double maxt;
			loadAttribute(pParent->ToElement(), maxt);
			msg.joints[countJoints-1].maxt = maxt;
		} else if(strcmp(pParent->Value(), "Kt") == 0)
		{
			double kt;
			loadAttribute(pParent->ToElement(), kt);
			msg.joints[countJoints-1].kt = kt;
		}
		break;

	case TiXmlNode::TINYXML_COMMENT:
		//printf( "Comment: [%s]", pParent->Value());
		break;

	case TiXmlNode::TINYXML_UNKNOWN:
		//printf( "Unknown" );
		break;

	case TiXmlNode::TINYXML_TEXT:
		pText = pParent->ToText();
		//printf( "Text: [%s]", pText->Value() );
		break;

	case TiXmlNode::TINYXML_DECLARATION:
		//printf( "Declaration" );
		break;
	default:
		break;
	}
	//printf("\n");
	for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling()) 
	{
		loadConfiguration( pChild, msg, countJoints);
	}
}

bool InitState::loadConfiguration(edo_core_msgs::JointConfigurationArray & msg, const char* pFilename)
{
  int jointsNum;
  TiXmlDocument doc(pFilename);
  bool loadOkay = doc.LoadFile();
  if (loadOkay)
  {
//    printf("\n%s:\n", pFilename);
    jointsNum = SPinstance->GetJointsNumber();
    if (jointsNum <= 0)
    {
      jointsNum = NUM_MAX_JOINTS;
    }
    msg.joints.resize(jointsNum);
    int count = 0;
    loadConfiguration( &doc, msg, count);
    ROS_INFO("Joints found in XML: %d", count);
  }
  else
  {
//    printf("Failed to load file \"%s\"\n", pFilename);
    return false;
  }

#if DEVELOPMENT_RELEASE
  for (auto i = msg.joints.begin(); i != msg.joints.end(); ++i)
    std::cout << *i << "\n";
#endif

  return true;
}

