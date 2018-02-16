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
 * edo_algorithms_node.cpp
 *
 *  Created on: 14/giu/2017
 *      Author: comaudev
 */

#include "edo_core_msgs/JointStateArray.h"
#include "edo_core_msgs/JointControlArray.h"
#include "edo_core_msgs/JointMonitoring.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstring>
#include <cstdio>
#include <climits>
#include <sstream>
/**
 * This function handles the movement message received from state machine node
 * - a movement command can not be executed until a valid jnt_state has been
 *   received from the recovery node
 */

ros::Publisher pub_error;

FILE *f_target;
FILE *f_real;

int scrivi = false;

edo_core_msgs::JointControlArray last_ctrl_joint_msg;

float distance_threashold = 5.0f;

void movementCallback(edo_core_msgs::JointControlArray msg)
{
	if (scrivi == true)
	{
	   struct timespec pino;
	   clock_gettime(CLOCK_REALTIME,&pino);
	   fprintf(f_target,"%ld.%09ld;",pino.tv_sec, pino.tv_nsec);

		for(int i =0 ;i<msg.joints.size();i++)
		{
			fprintf(f_target,"%f;%f;%f;",msg.joints[i].position,msg.joints[i].velocity,msg.joints[i].current);
		}
		fprintf(f_target,"\n");
	}

  // copia (allocato dinamicamente)
  last_ctrl_joint_msg = edo_core_msgs::JointControlArray(msg);
}

void statusCallback(edo_core_msgs::JointStateArray msg)
{
	if (scrivi == true)
	{
		struct timespec pino;
		clock_gettime(CLOCK_REALTIME,&pino);
		fprintf(f_real,"%ld.%09ld;",pino.tv_sec, pino.tv_nsec);

		for(int i =0 ;i<msg.joints.size();i++)
		{
			fprintf(f_real,"%f;%f;%f;",msg.joints[i].position,msg.joints[i].velocity,msg.joints[i].current);
		}
		fprintf(f_real,"\n");
	}

    for (int j = 0; j < last_ctrl_joint_msg.joints.size(); j++)
	{
		float dist = last_ctrl_joint_msg.joints[j].position - msg.joints[j].position;
		if (abs(dist) > distance_threashold) 
		{
			std::stringstream ss;
			ss << "error: on Joint " << j << " distance = " << dist;
			std_msgs::String msg;
			msg.data = ss.str();
			pub_error.publish(msg);
		}
    }
}

void moniCallback(edo_core_msgs::JointMonitoring msg)
{
	if (msg.state == 1 && scrivi == false)
	{ //apro il moni
		char name[100];
		time_t p = time(NULL);
		struct tm* seconds = localtime(&p);
		strftime(name, sizeof(name), "./%F_%H.%M.%S_", seconds);
		strcat(name,msg.name.c_str());
		strcat(name,"_real.txt");
		f_real=fopen(name,"w");

		strftime(name, sizeof(name), "./%F_%H.%M.%S_", seconds);
		strcat(name,msg.name.c_str());
		strcat(name,"_target.txt");
		f_target=fopen(name,"w");
		scrivi = true;
	}
	else
	{ // chiudo il moni
		fclose(f_real);
		fclose(f_target);
		scrivi = false;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv,"edo_recovery");
	ros::NodeHandle node_obj;
	ros::param::get("~th", distance_threashold);
	ros::Subscriber jnt_comm_subscriber = node_obj.subscribe("/algo_jnt_ctrl", 100, &movementCallback);
	ros::Subscriber jnt_state_subscriber = node_obj.subscribe("/usb_jnt_state", 100, &statusCallback);
	ros::Subscriber moni_subscriber = node_obj.subscribe("/machine_moni", 100, &moniCallback);

	pub_error = node_obj.advertise<std_msgs::String>("/edo_error", 10);

	ROS_INFO("Param %f", distance_threashold);

	ros::spin();

	return 0;
}
