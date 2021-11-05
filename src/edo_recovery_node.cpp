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
#include "CommonService.h"
using namespace CommonService;

#include "edo_recovery_node.hpp"

/**
 * This function handles the movement message received from state machine node
 * - a movement command can not be executed until a valid jnt_state has been
 *   received from the recovery node
 */
Edo_Recovery_Node::Edo_Recovery_Node(ros::NodeHandle& node_obj)
{
  distance_threshold_ = 5.0f;
  data_acquisition_in_progess_ = false;
  f_target_ = NULL;
  f_quote_ = NULL;
  n_max_target_samples_ = N_MAX_SAMPLES;
  n_max_quote_samples_ = N_MAX_SAMPLES;
  ros::param::get("~th", distance_threshold_);
  ros::Subscriber jnt_comm_subscriber = node_obj.subscribe("/algo_jnt_ctrl", 100, &Edo_Recovery_Node::movementCallback, this);
  ros::Subscriber jnt_state_subscriber = node_obj.subscribe("/usb_jnt_state", 100, &Edo_Recovery_Node::statusCallback, this);
  ros::Subscriber moni_subscriber = node_obj.subscribe("/machine_moni", 100, &Edo_Recovery_Node::moniCallback, this);

  pub_error = node_obj.advertise<std_msgs::String>("/edo_error", 10);

  ROS_INFO("Param %f", distance_threshold_);
  
  ros::spin();  // Endless loop

}

/**
 * Destructor of Edo_Recovery_Node
 * - releases data
 */
Edo_Recovery_Node::~Edo_Recovery_Node()
{
    data_acquisition_in_progess_ = false; // Disable data acquisition
    targetlist_.clear();
    quotelist_.clear();
    if (f_quote_ != NULL)
    {
      fclose(f_quote_);
      f_quote_ = NULL;
    }
    if (f_target_ != NULL)
    {
      fclose(f_target_);
      f_target_ = NULL;
    }
}



void Edo_Recovery_Node::movementCallback(edo_core_msgs::JointControlArray msg)
{
  if (data_acquisition_in_progess_ == true)
  {
    if (n_max_target_samples_ != 0)
    { 
      int    nJoints;
      
      --n_max_target_samples_;

      if ((nJoints = msg.joints.size()) <= SSM_NUM_MAX_JOINTS)
      {
        moniItem *mItemP = new moniItem;
        struct timespec timestamp;
        
        clock_gettime(CLOCK_REALTIME,&timestamp);
        mItemP->sec = timestamp.tv_sec;
        mItemP->nsec = timestamp.tv_nsec; 

        targetSize_ = nJoints;
        for(int i =0 ;i < nJoints;i++)
        {
          mItemP->dv[i].position = msg.joints[i].position;
          mItemP->dv[i].velocity = msg.joints[i].velocity;
      //mItemP->dv[i].velocity = msg.joints[i].R_rasp;
          mItemP->dv[i].current = msg.joints[i].current;
        }  
        targetlist_.push_back(*mItemP);
        
        // copia (allocato dinamicamente)
        last_ctrl_joint_msg_ = edo_core_msgs::JointControlArray(msg);
      }
    }
  }
  return;
}

void Edo_Recovery_Node::statusCallback(edo_core_msgs::JointStateArray msg)
{
  if (data_acquisition_in_progess_ == true)
  {
    if (n_max_quote_samples_ != 0) 
    {
      int    nJoints;
      
      --n_max_quote_samples_;

      if ((nJoints = msg.joints.size()) <= SSM_NUM_MAX_JOINTS)
      {
        moniItem *mItemP = new moniItem;
        struct timespec timestamp;
        
        clock_gettime(CLOCK_REALTIME,&timestamp);
        mItemP->sec = timestamp.tv_sec;
        mItemP->nsec = timestamp.tv_nsec; 

        quoteSize_ = nJoints;
        for(int i =0 ;i < nJoints;i++)
        {
          mItemP->dv[i].position = msg.joints[i].position;
          mItemP->dv[i].velocity = msg.joints[i].velocity;
      //mItemP->dv[i].velocity = msg.joints[i].R_jnt;
          mItemP->dv[i].current = msg.joints[i].current;
        }  
        quotelist_.push_back(*mItemP);
      }
      
      for (int j = 0; j < last_ctrl_joint_msg_.joints.size(); j++)
      {
        float dist = last_ctrl_joint_msg_.joints[j].position - msg.joints[j].position;
        if (abs(dist) > distance_threshold_) 
        {
          std::stringstream ss;
          ss << "error: on Joint " << j << " distance = " << dist;
          std_msgs::String msg;
          msg.data = ss.str();
          pub_error.publish(msg);
        }
      }
    }
  }
  return;
}

void Edo_Recovery_Node::moniCallback(edo_core_msgs::JointMonitoring msg)
{
  if (msg.state == 1)
  {
    // Do not re-activate if already active
    if (data_acquisition_in_progess_ == false)
    {  
      //open the data files
      char name[BUFSIZ];
      time_t p = time(NULL);
      struct tm* seconds = localtime(&p);
      
      name[0] = '\0';
      strftime(name, sizeof(name), "./%F_%H.%M.%S_", seconds);
      strcat(name,msg.name.c_str());
      strcat(name,"_real.txt");
      f_quote_=fopen(name,"w");
      
      name[0] = '\0';
      strftime(name, sizeof(name), "./%F_%H.%M.%S_", seconds);
      strcat(name,msg.name.c_str());
      strcat(name,"_target.txt");
      f_target_=fopen(name,"w");
      
      if ((f_target_ != NULL) && (f_quote_ != NULL))
      {
        data_acquisition_in_progess_ = true;  // Enable data acquisition
      }
      else
      {
        if (f_quote_ != NULL)
        {
          fclose(f_quote_);
          f_quote_ = NULL;
        }
        if (f_target_ != NULL)
        {
          fclose(f_target_);
          f_target_ = NULL;
        }
      }
    }
  }
  if (msg.state == 0)
  { 
    // Alway accept to close even if already closed.
    // close the data file and stop data acquisition
    data_acquisition_in_progess_ = false; // Disable data acquisition
    
    if ((!targetlist_.empty()) && (f_target_ != NULL))
    {
      for (std::list<moniItem>::iterator it=targetlist_.begin(); it != targetlist_.end(); ++it)
      {
        fprintf (f_target_,"%ld.%09ld;", it->sec, it->nsec);
        for(int i =0 ;i < targetSize_;i++) 
        {
          fprintf(f_target_,"%f;%f;%f;",it->dv[i].position,it->dv[i].velocity,it->dv[i].current);
        }
        fprintf(f_target_,"\n");
      }
      targetlist_.clear();
    }
    if ((!quotelist_.empty()) && (f_quote_ != NULL))
    {
      for (std::list<moniItem>::iterator it=quotelist_.begin(); it != quotelist_.end(); ++it)
      {
        fprintf (f_quote_,"%ld.%09ld;", it->sec, it->nsec);
        for(int i =0 ;i < quoteSize_;i++) 
        {
          fprintf(f_quote_,"%f;%f;%f;",it->dv[i].position,it->dv[i].velocity,it->dv[i].current);
        }
        fprintf(f_quote_,"\n");
      }
      quotelist_.clear();
    }
    
    if (f_quote_ != NULL)
    {
      fclose(f_quote_);
      f_quote_ = NULL;
    }
    if (f_target_ != NULL)
    {
      fclose(f_target_);
      f_target_ = NULL;
    }
    n_max_target_samples_ = N_MAX_SAMPLES;
    n_max_quote_samples_ = N_MAX_SAMPLES;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv,"edo_recovery");
  ros::NodeHandle node_obj;
  Edo_Recovery_Node recovery(node_obj);

  return 0;
}