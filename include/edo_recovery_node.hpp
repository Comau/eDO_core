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
#ifndef EDO_CORE_PKG_INCLUDE_EDORECOVERYNODE_HPP_
#define EDO_CORE_PKG_INCLUDE_EDORECOVERYNODE_HPP_

#include "edo_core_msgs/JointStateArray.h"
#include "edo_core_msgs/JointControlArray.h"
#include "edo_core_msgs/JointMonitoring.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstring>
#include <cstdio>
#include <climits>
#include <sstream>

class Edo_Recovery_Node
{
public:

#define N_MAX_SAMPLES (300 * 100)  // 300s of samples (100 samples per second)  

	Edo_Recovery_Node(ros::NodeHandle&);
	~Edo_Recovery_Node();
  void moniCallback(edo_core_msgs::JointMonitoring msg);
  void statusCallback(edo_core_msgs::JointStateArray msg);
  void movementCallback(edo_core_msgs::JointControlArray msg);
  
private:

  struct dataValues {
    float position;
    float velocity;
    float current;
  };
  
  struct moniItem {
    time_t   sec;
    long int nsec;
    dataValues dv[SSM_NUM_MAX_JOINTS];
  };
  
  ros::Publisher pub_error;

  FILE *f_target_;
  FILE *f_quote_;

  bool data_acquisition_in_progess_;
  unsigned int n_max_target_samples_;
  unsigned int n_max_quote_samples_;
  edo_core_msgs::JointControlArray last_ctrl_joint_msg_;

  float distance_threshold_;
  int                 targetSize_;
  int                 quoteSize_;
	std::list<moniItem> targetlist_;
	std::list<moniItem> quotelist_;
  
};

#endif