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
 * TabletCheck.cpp
 *
 *  Created on: 10 Apr 2019
 *      Author: comau
 */
 #define HB_THR 1
 #define ENABLE_ROS_INFO  (1==0)
 
 #include "TabletCheck.hpp"
 
 TabletCheck::TabletCheck(ros::NodeHandle& node) 
{
  _counter = 0;
  _hb_Flag = false;
  
  tablet_HB_subscriber       = node.subscribe("tablet_HB", 10, &TabletCheck::hb_Callback, this);
  tablet_HB_start_subscriber = node.subscribe("tablet_HB_start", 200, &TabletCheck::hb_start_Callback, this);
  tablet_ACK_publisher       = node.advertise<std_msgs::Bool>("tablet_ACK", 10);
  
  hb_Timer = node.createTimer(ros::Duration(1.0), &TabletCheck::hb_TimerCallback, this, false, false);
}

TabletCheck::~TabletCheck() 
{
  #if ENABLE_ROS_INFO
  ROS_INFO("Stop Tablet Check");
  #endif
}

void TabletCheck::hb_Callback(std_msgs::Int8 msg)
{
  if(_hb_Flag)
  {
    #if ENABLE_ROS_INFO
    ROS_INFO("Received beat");
    #endif
    
    _counter++;
    // counter = counter + msg.data;     //ricevo +1 -1 alternativamente
  }
}

void TabletCheck::hb_start_Callback(std_msgs::Bool msg)
{
  if(msg.data)
  {
    #if ENABLE_ROS_INFO
    ROS_INFO("Start Tablet Check");
    #endif
    
    _counter = 0;
    hb_Timer.start();
    _hb_Flag = true;
  }
  else
  {
    #if ENABLE_ROS_INFO
    ROS_INFO("Stop Tablet Check");
    #endif
    
    _counter = 0;
    hb_Timer.stop();
    _hb_Flag = false;
  }
}

void TabletCheck::hb_TimerCallback(const ros::TimerEvent& event)
{
  std_msgs::Bool msg;
  //printf("contatore: %d\n", _counter);
   
  if(_counter < HB_THR)
  {
    #if ENABLE_ROS_INFO
    ROS_INFO("Fail");
    #endif
    
    msg.data = true;
    tablet_ACK_publisher.publish(msg);
    
    hb_Timer.stop();
    _hb_Flag = false;
  }
  _counter = 0;
}