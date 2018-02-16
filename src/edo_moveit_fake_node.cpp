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

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include "edo_core_msgs/JointStateArray.h"
#include "edo_core_msgs/MovementCommand.h"
#include <angles/angles.h>
#include <geometry_msgs/Pose.h>
#include <vector>

// This is the topic where the node sends the joints state
ros::Publisher joint_state_publisher;

// This is the topic where the node subscribes to usb joint state (from rosserial/usb node)
ros::Subscriber usb_jnt_state_subscriber;

// This is the topic where the node sends the bridge move
ros::Publisher bridge_move_publisher;

// This is the topic where the node subscribes to move group
ros::Subscriber move_to_target_subscriber;


void UsbJointState(const edo_core_msgs::JointStateArray msg) {

	sensor_msgs::JointState joint_state = sensor_msgs::JointState();
	
	for (std::size_t i = 0; i < msg.joints.size(); i++)
	{
		std::string name = "axes_" + std::to_string(i+1);
		joint_state.name.push_back(name);
		joint_state.position.push_back(angles::from_degrees(msg.joints[i].position));
		joint_state.velocity.push_back(angles::from_degrees(msg.joints[i].velocity));
		joint_state.effort.push_back(msg.joints[i].current);
	}
	
	joint_state_publisher.publish(joint_state);
}

void MoveToTarget(const sensor_msgs::JointState group_variable_values) {

	edo_core_msgs::MovementCommand msg;

	msg.movement_type = 0; // joint space
	msg.size  = group_variable_values.position.size();
	msg.ovr = 0;
	for (std::size_t i = 0; i < group_variable_values.position.size(); i++)
	{
		msg.data.push_back(angles::to_degrees(group_variable_values.position[i])); //float32[] data
	}
	//msg.uint8[] movement_attributes
	
	bridge_move_publisher.publish(msg);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv,"edo_moveit_fake");
	ros::NodeHandle node_obj;
	ros::Rate loop_rate(100);
	
	joint_state_publisher = node_obj.advertise<sensor_msgs::JointState>("/edo_joint_state",10);
	usb_jnt_state_subscriber = node_obj.subscribe("/usb_jnt_state",10, &UsbJointState);
	bridge_move_publisher = node_obj.advertise<edo_core_msgs::MovementCommand>("/bridge_move",10);
	move_to_target_subscriber = node_obj.subscribe("/joint_move",10, &MoveToTarget);
	
	while(ros::ok())
	{
	      ros::spinOnce();
	      loop_rate.sleep();
	}

	return 0;
}

