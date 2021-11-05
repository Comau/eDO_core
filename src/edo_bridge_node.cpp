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
#include <iostream>
#include <cstdio>
#include "ros/ros.h"
//#include "ros/rosbridge_library.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "edo_core_msgs/JointControlArray.h"
#include "edo_core_msgs/JointStateArray.h"
#include "edo_core_msgs/JointCalibration.h"
#include "edo_core_msgs/JointConfigurationArray.h"
#include "edo_core_msgs/JointInit.h"
#include "edo_core_msgs/JointReset.h"
#include "edo_core_msgs/MovementCommand.h"
#include "EdoMsgType.h"

// -------------- TOPIC FROM/TO BRIDGE NODE --------------
// This is the topic where the bridge node publishes reset command
ros::Publisher bridge_jnt_reset_publisher;
// This is the topic where the bridge node publishes configuration command
ros::Publisher bridge_jnt_config_publisher;
// This is the topic where the bridge node publishes calibration command
ros::Publisher bridge_jnt_calib_publisher;
// This is the topic where the bridge node publishes move commands
ros::Publisher bridge_move_publisher;
// This is the topic where the bridge node publishes jog commands
ros::Publisher bridge_jog_publisher;
// This is the topic where the bridge node publishes init commands
ros::Publisher bridge_init_publisher;
// This is the service where the node receives the joints state
// ros::Subscriber machine_bridge_jnt_state_subscriber;
// This is the service where the node receives the state machine state
// ros::Subscriber machine_state_subscriber;

#define DEVELOPMENT_RELEASE (1==0)

void PerformConfiguration()
{
    edo_core_msgs::JointConfigurationArray config_msg;
  int size;
  // edo_core_msgs::JointConfigurationArray msg;
  config_msg.joints.resize(1); //HARD CODED

  fflush(stdin);
  printf("Please provide joint configuration parameters [format single_joint_mask:kp:ti:td:sat:kff:max:kpv:tiv:tdv:satv:kffv:maxv:kpt:tit:tdt:satt:kfft:maxt]: ");
    size = scanf("%lld:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f:%f",
      &config_msg.joints_mask,
      &config_msg.joints[0].kp,
      &config_msg.joints[0].ti,
      &config_msg.joints[0].td,
      &config_msg.joints[0].sat,
      &config_msg.joints[0].kff,
      &config_msg.joints[0].max,
      &config_msg.joints[0].kpv,
      &config_msg.joints[0].tiv,
      &config_msg.joints[0].tdv,
      &config_msg.joints[0].satv,
      &config_msg.joints[0].kffv,
      &config_msg.joints[0].maxv,
      &config_msg.joints[0].kpt,
      &config_msg.joints[0].tit,
      &config_msg.joints[0].tdt,
      &config_msg.joints[0].satt,
      &config_msg.joints[0].kfft,
      &config_msg.joints[0].maxt);

  if (size != 19)
  {
    printf("wrong format\n");
    return;
  }

  printf("Send msg\n");
  bridge_jnt_config_publisher.publish(config_msg);
  printf("msg sent\n");
}

void PerformInit() {
  edo_core_msgs::JointInit init_msg;
  int size;

  fflush(stdin);
  printf("Please provide joint init parameters [format mode:joint_mask:reduction_factor]: ");
  size = scanf("%hhu:%lld:%f",
      &init_msg.mode,
      &init_msg.joints_mask,
      &init_msg.reduction_factor);


  if (size != 3)
  {
#if DEVELOPMENT_RELEASE
    printf("wrong format\n");
#endif
    return;
  }

  bridge_init_publisher.publish(init_msg);
}

void PerformReset() {
  edo_core_msgs::JointReset reset_msg;
  int size;

  fflush(stdin);
  printf("Please provide joint reset parameters [format joint_mask]: ");
  size = scanf("%lld", &reset_msg.joints_mask );

  if (size != 1)
  {
#if DEVELOPMENT_RELEASE
    printf("wrong format\n");
#endif
    return;
  }

  bridge_jnt_reset_publisher.publish(reset_msg);
}

void PerformCalibration() {
  edo_core_msgs::JointCalibration calib_msg;
  int size;

  fflush(stdin);
  printf("Please provide joint calibration parameters [formatjoint_mask]: ");
  size = scanf("%lld", &calib_msg.joints_mask );

  if (size != 1)
  {
    printf("wrong format\n");
    return;
  }

  bridge_jnt_calib_publisher.publish(calib_msg);
}


void PerformJog() {
  edo_core_msgs::MovementCommand move_msg;
  int nrItems, nrJoints, attributes = 0;

  fflush(stdin);
  printf("Please provide joint Jog parameters [format JogMoveType:NrJoints]: ");
  nrItems = scanf("%hhu:%d", &move_msg.move_type, &nrJoints );

  if (nrItems != 2)
  {
#if DEVELOPMENT_RELEASE
    printf("wrong format\n");
#endif
    return;
  }
  if (move_msg.move_type != E_MOVE_TYPE::E_MOVE_TYPE_JOINT)
  {
#if DEVELOPMENT_RELEASE
    printf("wrong MOVE type\n");
#endif
    return;
  }
  move_msg.move_command = E_MOVE_COMMAND::E_MOVE_COMMAND_MOVE;
  move_msg.delay = 0;
  move_msg.ovr   = 100;
  move_msg.target.joints_data.resize(nrJoints);

  for (int i = 0; i < nrJoints; i++) {
    printf("Please provide joint %d data: ", i+1);
    nrItems = scanf("%f", &move_msg.target.joints_data[i]);
    if (nrItems == 1)
    {
      move_msg.target.joints_mask |= (1 << i);
    }
  }
#if 0
  printf("Please provide the number of movement attributes: ");
  nrItems = scanf("%d", &attributes );

  if ((nrItems == 1) && (attributes > 0)) {

    move_msg.movement_attributes.resize(attributes);

    for (int i = 0; i < attributes; i++) {
      printf("Please provide attributes %d: ", i+1);
      nrItems = scanf("%d", &move_msg.movement_attributes[i] );
    }
  }
#endif
  bridge_jog_publisher.publish(move_msg);
}

void PerformMove() {
  edo_core_msgs::MovementCommand move_msg;
  int nrItems, nrJoints, attributes = 0;

  fflush(stdin);
  printf("Please provide joint Move parameters [format MoveType:NrJoints]: ");
  nrItems = scanf("%hhu:%d", &move_msg.move_type, &nrJoints);
  if (nrItems != 2)
  {
    printf("wrong format\n");
    return;
  }
  if (move_msg.move_type != E_MOVE_TYPE::E_MOVE_TYPE_JOINT)
  {
#if DEVELOPMENT_RELEASE
    printf("wrong MOVE type\n");
#endif
    return;
  }

  move_msg.move_command = E_MOVE_COMMAND::E_MOVE_COMMAND_MOVE;
  move_msg.delay = 0;
  move_msg.ovr   = 100;
  move_msg.target.joints_data.resize(nrJoints);

  for (int i = 0; i < nrJoints; i++) {
    printf("Please provide joint %d data: ", i+1);
    nrItems = scanf("%f", &move_msg.target.joints_data[i] );
    if (nrItems == 1)
    {
      move_msg.target.joints_mask |= (1 << i);
    }
  }
#if 0
  printf("Please provide the number of movement attributes: ");
  nrItems = scanf("%d", &attributes );

  if ((nrItems == 1) && (attributes > 0)) {

    move_msg.movement_attributes.resize(attributes);

    for (int i = 0; i < attributes; i++) {
      printf("Please provide attributes %d: ", i+1);
      nrItems = scanf("%d", &move_msg.movement_attributes[i] );
    }
  }
#endif
  bridge_move_publisher.publish(move_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv,"edo_bridge");

  ros::NodeHandle node_obj;

  bridge_jnt_reset_publisher = node_obj.advertise<edo_core_msgs::JointReset&>("/bridge_jnt_reset",10);
  bridge_jnt_config_publisher = node_obj.advertise<edo_core_msgs::JointConfigurationArray&>("/bridge_jnt_config",10);
  bridge_jnt_calib_publisher = node_obj.advertise<edo_core_msgs::JointCalibration>("/bridge_jnt_calib",10);
  bridge_move_publisher = node_obj.advertise<edo_core_msgs::MovementCommand>("/bridge_move",10);
  bridge_jog_publisher = node_obj.advertise<edo_core_msgs::MovementCommand>("/bridge_jog",10);
  bridge_init_publisher = node_obj.advertise<edo_core_msgs::JointInit&>("/bridge_init",10);
  //machine_bridge_jnt_state_subscriber;
  //machine_state_subscriber;

  unsigned int command_type;
  while (ros::ok())
  {
    command_type = 9;
      printf("set command type: [0] for init, [1] for config, [2] for reset, [3] for calibration, [4] to jog, [5] to move, [6] to quit:");
      scanf("%d", &command_type);

    if(command_type > 6)
    {
#if DEVELOPMENT_RELEASE
      printf("Unrecognized command\n");
#endif
    }
    else if(command_type == 0)
      PerformInit();
    else if(command_type == 1)
      PerformConfiguration();
    else if(command_type == 2)
      PerformReset();
    else if(command_type == 3)
      PerformCalibration();
    else if(command_type == 4)
      PerformJog();
    else if(command_type == 5)
      PerformMove();
    else if(command_type == 6) {
#if DEVELOPMENT_RELEASE
      printf("Exit\n");
#endif
      return 0;
    }
  }

    return 0;
}

