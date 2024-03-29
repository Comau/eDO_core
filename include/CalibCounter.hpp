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
 * CalibCounter.hpp
 *
 *  Created on: Jun 26, 2017
 *      Author: comau
 */

#ifndef EDO_CORE_PKG_INCLUDE_CALIBCOUNTER_HPP_
#define EDO_CORE_PKG_INCLUDE_CALIBCOUNTER_HPP_

#include "edo_core_msgs/CalibCounter.h"
#include "ros/ros.h"
#include <ros/package.h>

class CalibCounter
{

public:
  
  CalibCounter(ros::NodeHandle&);
  ~CalibCounter();
  
  // Public Methods
  bool getCalibrationCounter(edo_core_msgs::CalibCounter::Request &req, edo_core_msgs::CalibCounter::Response &res);
  bool incrementCalibrationCounter(edo_core_msgs::CalibCounter::Request &req, edo_core_msgs::CalibCounter::Response &res);
  
private:
    ros::ServiceServer calibration_counter_service;
    ros::ServiceServer increment_calib_counter_service;
};

#endif /* EDO_CORE_PKG_INCLUDE_CALIBCOUNTER_HPP_ */
