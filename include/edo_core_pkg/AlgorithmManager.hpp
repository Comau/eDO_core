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
 * AlgorithmManager.hpp
 *
 *  Created on: Jun 26, 2017
 *      Author: comau
 */

#ifndef EDO_CORE_PKG_INCLUDE_ALGORITHMMANAGER_HPP_
#define EDO_CORE_PKG_INCLUDE_ALGORITHMMANAGER_HPP_

#include <pthread.h>
#include <vector>
#include <cstdint>
#include <math.h>
#include <cstring>
#include <cstdio>
#include <climits>
#include <list>
#include <algorithm>
#include <cstdlib>

#include "edo_core_msgs/JointStateArray.h"
#include "edo_core_msgs/JointControlArray.h"
#include "edo_core_msgs/MovementCommand.h"
#include "edo_core_msgs/MovementFeedback.h"
#include "edo_core_msgs/CartesianPose.h"
#include "edo_core_msgs/JointsNumber.h"
#include "edo_core_msgs/JointsPositions.h"
#include "edo_core_msgs/DirectKinematics.h"
#include "edo_core_msgs/InverseKinematics.h"
#include "edo_core_msgs/JointCalibration.h"
#include "edo_core_msgs/ControlSwitch.h"
#include "edo_core_msgs/LoadConfigurationFile.h"
#include "edo_core_msgs/CollisionFromAlgoToState.h"
#include <std_msgs/Int8.h>
#include "ros/ros.h"
#include <ros/package.h>

#include "eORL.h"
#include "EdoState.h"
#include "EdoMsgType.h"

#include <boost/circular_buffer.hpp>


#define MAN_LIN_STEP 10
#define MAN_ORN_STEP 0.1

#define IDX_STRK_ENP_P   1
#define IDX_STRK_ENP_N   0

class AlgorithmManager
{
public:

#define INTERPOLATION_STEP      2  // GET_NEXT_STEP CALLS PERIOD
#define INTERPOLATION_TIME_STEP (INTERPOLATION_STEP / 1000.0f)  // Ts in ms
#define CONTROLLER_FREQUENCY    100 // in Hz
#define CONTROLLER_PERIOD         (1000.0f / CONTROLLER_FREQUENCY) // A target is produced every 10ms
#define NR_GET_NEXT_STEP_CALLS    (CONTROLLER_PERIOD / INTERPOLATION_STEP)
// INTERPOLATION_STEP --> 1  DUE_PER_TS --> 0.002  TS_SQUARE --> 0.000001
// INTERPOLATION_STEP --> 2  DUE_PER_TS --> 0.004  TS_SQUARE --> 0.000004
#define DUE_PER_TS                (2.0f * INTERPOLATION_TIME_STEP)  
#define TS_SQUARE                 (INTERPOLATION_TIME_STEP * INTERPOLATION_TIME_STEP)
#define ORL_STATUS_IDLE -42
#define FILTER_STEP             3
#define N_AX_GRIPPER            6

#define KF_THRS_IST1   		0.5f
#define KF_THRS_IST2   		1.0f
#define KF_THRS_IST3   		0.8f
#define KF_THRS_IST4   		0.4f
#define KF_THRS_IST5   		0.5f
#define KF_THRS_IST6   		0.3f

#define ZERO_FLOAT			0.0f
#define ONE_FLOAT			1.0f
#define TWO_FLOAT			2.0f
#define PI_GRECO			3.14159265358979f
#define QUASIZERO_FLOAT 	0.00000001f

	enum Mode {
		UNINITIALIZED = 0,
		INITIALIZED = 1,
		MOVING = 2,
		WAITING = 3,
		BLOCKED = 4,
		FINISHED = 5,
		PAUSE = 6,
		RECOVERY = 7,
		HOLD = 8,
		SWITCHED_OFF = 9,
		MAX_NUM_ALGORITHM_MODES = 10
	};
  
  enum COMMAND_FLAG {
    IDLE            = 0x00,
    /* the Ack code is in the first nibble */
    ACK_INIT        = 1,    /* ack a un messaggio di init */
    ACK_CALIBRATION = 2,    /* ack a un messaggio di calibrazione */
    ACK_CONFIG      = 3,    /* ack a un messaggio di configurazione pid */
    ACK_RESET       = 4,    /* ack a un messaggio di reset */
    ACK_FW_VERSION  = 5,    /* ack a un messaggio di firmware version */
    /* the Error status bits are coded in the second nibble */
    COLLISION       = 4,    /* bit 4 - Rilevata collisione */
    H_BRIDGE_DOWN   = 5,    /* bit 5 - ponte h aperto - no potenza motori */
    OVERCURRENT     = 6,    /* bit 6 - sovracorrente */
    UNCALIBRATED    = 7,    /* bit 7 - giunto non calibrato */
    // Mask of the Error status bits
    STATUS_ERROR_MASK = 0xF0
  };
	AlgorithmManager(ros::NodeHandle&);
	~AlgorithmManager();

	void moveCallback(edo_core_msgs::MovementCommandConstPtr);
	void jogCallback(edo_core_msgs::MovementCommandConstPtr);
	edo_core_msgs::JointControlArray const& getCurrentControl();
	void stateCallback(edo_core_msgs::JointStateArrayConstPtr);
	void updateControl();

private:
	ros::Timer timerCalib_;
	Mode algorithm_mode_;
	ORL_joint_value current_state_;
	ORL_joint_value target_joint_;
	ORL_joint_value target_joint_via_;
	ORL_joint_value hold_position_;
	ORL_Dynamic_Model hold_current_;
	ORL_Dynamic_Model sx_dyn_mod_;
	ORL_cartesian_position target_cart_;
	ORL_cartesian_position target_cart_via_;
	ORL_cartesian_position base_;
	ORL_cartesian_position tool_;
	ORL_cartesian_position uframe_;
	ORL_cartesian_position p_;
	boost::circular_buffer<ORL_joint_value> interpolation_data_;
 	double controller_frequency_, state_saturation_threshold_;
	int joints_number_;
	int interpolation_time_step_;
	int  jog_target_data_type_;
	int aggPosCartPose_;
	int configurationFileLoaded_;
	int numberSteps_;
	int numberSteps_M42_;
	int strk_[2][ORL_MAX_AXIS];
	int previousMoveFlyHandle_;
	int idleMoveHandle_;
	uint delay_;
	bool waiting_, jog_carlin_state_, jog_state_;
	bool  noCartPose_;
	bool first_time_;
	bool pause_state_;
	bool alarm_state_;
	bool alarm_exclusion_;
	bool alarm_state_ack_;
	bool pending_cancel_;
	bool moveInProgress_;
	bool moveStopped_;
	ros::Time start_wait_time_;
	ros::Time start_jog_time_;
	pthread_mutex_t control_mutex_;
	unsigned long joints_mask_;
	unsigned long joints_aux_mask_;
	uint32_t jointCalib_;
	std::string pkg_path_;
	float R_old_[3][3], R_step_[3][3];
	edo_core_msgs::JointControlArray joints_control_;
	edo_core_msgs::MovementFeedback next_move_request_;
	edo_core_msgs::MovementCommand in_progress_moveCommand_;
	std::list<edo_core_msgs::MovementCommand> msglist_;
	std::list<edo_core_msgs::MovementCommand> msglistJog_;
  std::list<edo_core_msgs::MovementCommand> MoveCommandMsgListInOrl_;
	ros::Publisher feedback_publisher_, cartesian_pose_pub_, algorithm_state_pub_, algo_collision_publisher;

	bool moveTrjntFn    (edo_core_msgs::MovementCommand * msg);
	bool moveCarlinFn   (edo_core_msgs::MovementCommand * msg);
	bool moveCarcirFn   (edo_core_msgs::MovementCommand * msg);
	bool movePauseFn    (edo_core_msgs::MovementCommand * msg);
	bool moveResumeFn   (edo_core_msgs::MovementCommand * msg);
	bool moveCancelFn   (edo_core_msgs::MovementCommand * msg);
	bool jogTrjntFn     (edo_core_msgs::MovementCommand * msg);
	bool jogCarlinFn    (edo_core_msgs::MovementCommand * msg);
	bool jogStopFn      (edo_core_msgs::MovementCommand * msg);
	bool setReferenceFn (edo_core_msgs::MovementCommand * msg);
	void calibCallback(edo_core_msgs::JointCalibrationConstPtr msg);
	void timerCallback(const ros::TimerEvent& event);
	Mode getAlgorithmMode(void) { return(algorithm_mode_); };
	void setAlgorithmMode(Mode si_mode, const char *apc_func, int si_line);
	void initialize (edo_core_msgs::JointStateArrayConstPtr);
	int setORLMovement(std::vector<int> move_parameters, int movement_type, int point_data_type, int ovr);
	void setControl(bool bactive);
	void keepPosition();
	bool manageORLStatus(int const&, const char* service_name);
	int get_Strk(int strk[2][ORL_MAX_AXIS]);
	void vZYZm(ORL_cartesian_position p, float R[3][3]);
	void mvZYZ(float R[3][3], ORL_cartesian_position *p);
	void vXYZm(ORL_cartesian_position p, float R[3][3]);
	edo_core_msgs::CartesianPose computeCartesianPose(ORL_joint_value *joint_state);
	bool updateJogCarlin();
	bool getSrvJointsNumber(edo_core_msgs::JointsNumber::Request  &req, edo_core_msgs::JointsNumber::Response &res);
	bool getDirectKinematics(edo_core_msgs::DirectKinematics::Request  &req, edo_core_msgs::DirectKinematics::Response &res);
	bool getInverseKinematics(edo_core_msgs::InverseKinematics::Request  &req, edo_core_msgs::InverseKinematics::Response &res);
	bool loadConfigurationFile_CB(edo_core_msgs::LoadConfigurationFile::Request  &req, edo_core_msgs::LoadConfigurationFile::Response &res);
	void feedbackFn();
	bool initializeORL();
	void feedbackFn(int type, int data, const char *, int);
	edo_core_msgs::JointsPositions computeJointValue(edo_core_msgs::CartesianPose cartesian_pose);
	
	ros::ServiceServer robot_switch_control_server;
	bool SwitchControl(edo_core_msgs::ControlSwitch::Request &req, edo_core_msgs::ControlSwitch::Response &res);
	bool collisionCheck(float vr_CurMis, float vr_CurDyn, int vi_JointIndex);
	
	//------- COLLISION DETECTION ------//
    int        _curfilt_cnt[6]={0}      ;
    float _FiltCurDyn_Regr2[6]={0.0f} ;
    float _FiltCurDyn_Regr1[6]={0.0f} ;
    float _FiltCurMis_Regr2[6]={0.0f} ;
    float _FiltCurMis_Regr1[6]={0.0f} ;
	
};



#endif /* EDO_CORE_PKG_INCLUDE_ALGORITHMMANAGER_HPP_ */
