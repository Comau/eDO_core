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
 * CalibCounter.cpp
 *
 *  Created on: Jun 26, 2017
 *      Author: comau
 */
#include "CalibCounter.hpp"
  
CalibCounter::CalibCounter(ros::NodeHandle& node)
{
    ROS_INFO("created calib counter service");
    calibration_counter_service = node.advertiseService("calibration_counter_srv", &CalibCounter::getCalibrationCounter, this);
    increment_calib_counter_service = node.advertiseService("increment_calib_counter_srv", &CalibCounter::incrementCalibrationCounter, this);
}

/**
 * Destructor of CalibCounter
 * - releases ORL library
 */
CalibCounter::~CalibCounter()
{
#if ENABLE_ROS_INFO
  ROS_INFO("Terminate calibCounter...");
#endif
}

bool CalibCounter::incrementCalibrationCounter(edo_core_msgs::CalibCounter::Request &req, edo_core_msgs::CalibCounter::Response &res)
{

  /* Count calibrations */
  FILE *cnt;
  int updatedCnt = 1;
  int oldCnt = 0;
  int maxCounter = 1000000;
  
  if (req.increment == false) {
      updatedCnt = 2;
  } else if( (cnt=fopen("/home/edo/calib.log", "r+")) != NULL) {
      // file exists
      oldCnt = getw(cnt);
      updatedCnt = oldCnt + 1;
      fclose(cnt);
  }

  if (updatedCnt >= maxCounter) {
      updatedCnt = 0;
  }

  cnt = fopen("/home/edo/calib.log", "w");
  putw(updatedCnt,cnt);
  fclose(cnt);

  ROS_INFO("Calibration N %d",updatedCnt);
  return true;
}

bool CalibCounter::getCalibrationCounter(edo_core_msgs::CalibCounter::Request &req, edo_core_msgs::CalibCounter::Response &res)
{
    /* Count calibrations */
    FILE *cnt;
    int counter = 0;
    ROS_INFO("get calibration counter start");

    if( (cnt=fopen("/home/edo/calib.log", "r+")) != NULL) {
      // file exists
      counter = getw(cnt);
      fclose(cnt);
    }
    res.cnt = counter;
    ROS_INFO("Read calibration counter: %d",counter);
    return true;

}