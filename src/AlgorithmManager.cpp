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
 * AlgorithmManager.cpp
 *
 *  Created on: Jun 26, 2017
 *      Author: comau
 */

#include "edo_core_pkg/AlgorithmManager.hpp"
#include "CommonService.h"
using namespace CommonService;

#define DEVELOPMENT_RELEASE (1==0)

#if DEVELOPMENT_RELEASE
/**this sets verbosity level of ORL library**/
#define LOCAL_VERBOSITY ORL_VERBOSE /**** ORL_VERBOSE or **ORL_SILENT**/
#define ENABLE_ROS_WARN  (1==0)
#define ENABLE_ROS_INFO  (1==0)
#define ENABLE_ALGOMNGR_PRINTFS (1==0)
#define ENABLE_MINIMAL_ALGOMNGR_PRINTFS (1==0)
#define ENABLE_DYNAMIC_MODEL_PRINTFS (1==0)
#else
/**this sets verbosity level of ORL library**/
#define LOCAL_VERBOSITY ORL_SILENT /**** ORL_VERBOSE or **ORL_SILENT**/
#define ENABLE_ROS_WARN  (1==0)
#define ENABLE_ROS_INFO  (1==0)
#define ENABLE_ALGOMNGR_PRINTFS (1==0)
#define ENABLE_MINIMAL_ALGOMNGR_PRINTFS (1==0)
#define ENABLE_DYNAMIC_MODEL_PRINTFS (1==0)
#endif  

#define ENABLE_KINEMATICS_SERVICES (1==0)
/**
 * Constructor of the manager class writes algorithm
 *  - sets the position of the configuration file (WARNING HARD CODED bad habit)
 *  - initialize ORL library
 *  - Basic handling of errors from ORL @TODO Signal a fatal error to the state
 *    machine.
 */ 
  
AlgorithmManager::AlgorithmManager(ros::NodeHandle& node):
algorithm_mode_(UNINITIALIZED)
{
	ros::NodeHandle private_nh("~");
	edo_core_msgs::JointsNumber::Request req;
	edo_core_msgs::JointsNumber::Response res;
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
	ros::Time     last_publish_time;
	ros::Duration elapsed_publish_time;
	double elapsed_time_in_msec;
#endif

	//subscribe to /movement_comm, published by state machine and /jnt_state from rosserial
	ros::Subscriber move_control_sub = node.subscribe("machine_move", 100, &AlgorithmManager::moveCallback, this);
	ros::Subscriber jog_control_sub = node.subscribe("machine_jog", 100, &AlgorithmManager::jogCallback, this);
	ros::Subscriber jnt_state_subscriber = node.subscribe("machine_algo_jnt_state", 100, &AlgorithmManager::stateCallback, this);
	ros::Subscriber jnt_calib_subscriber = node.subscribe("machine_jnt_calib", 100, &AlgorithmManager::calibCallback, this);
	ros::Subscriber machine_state_subscriber = node.subscribe("machine_state", 100, &AlgorithmManager::machineStateCallback, this);
	ros::Subscriber machine_jnt_config_subscriber = node.subscribe("machine_jnt_config", 100 , &AlgorithmManager::jntConfigCallback, this);
	ros::Subscriber machine_jnt_reset_subscriber = node.subscribe("machine_jnt_reset", 100 , &AlgorithmManager::jntResetCallback, this);
	
	// publish topic
	algo_collision_publisher = node.advertise<edo_core_msgs::CollisionAlgoToState>("algo_collision", 10);	
	feedback_publisher_ = node.advertise<edo_core_msgs::MovementFeedback>("algo_movement_ack", 200);
	cartesian_pose_pub_ = node.advertise<edo_core_msgs::CartesianPose>("cartesian_pose", 100);
	algorithm_state_pub_ = node.advertise<std_msgs::Int8>("algorithm_state", 100);

	//create a ROS Service Server
	ros::ServiceServer get_jnts_number_srv = node.advertiseService("algo_jnt_number_srv", &AlgorithmManager::getSrvJointsNumber, this);
#if ENABLE_KINEMATICS_SERVICES
	ros::ServiceServer get_direct_kinematics_srv = node.advertiseService("algo_direct_kinematics_srv", &AlgorithmManager::getDirectKinematics, this);
	ros::ServiceServer get_inverse_kinematics_srv = node.advertiseService("algo_inverse_kinematics_srv", &AlgorithmManager::getInverseKinematics, this);
#endif
	ros::ServiceServer loadConfigurationFile_srv = node.advertiseService("algo_load_configuration_file_srv", &AlgorithmManager::loadConfigurationFile_CB, this);
	robot_switch_control_server = node.advertiseService("algo_control_switch_srv", &AlgorithmManager::SwitchControl, this);

	//publish /jnt_ctrl to rosserial, then joints
	ros::Publisher robot_control_publisher = node.advertise<edo_core_msgs::JointControlArray>("algo_jnt_ctrl", 100);
	
	//subscribe to 
	ros::Subscriber coll_thr_subscriber = node.subscribe("algo_coll_thr", 100, &AlgorithmManager::algoCollisionThr, this);
	
	//memset(&next_move_request_, 0, sizeof(edo_core_msgs::MovementFeedback));
	memset(&target_joint_, 0, sizeof(ORL_joint_value));
	memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
	memset(&hold_position_, 0, sizeof(ORL_joint_value));
	memset(&current_state_, 0, sizeof(ORL_joint_value));
	memset(&hold_current_, 0, sizeof(ORL_Dynamic_Model));
	memset(&sx_dyn_mod_, 0, sizeof(ORL_Dynamic_Model));
	delay_ = 255;
	waiting_ = false;
	jog_state_ = false;
	pending_cancel_ = false;
	jog_carlin_state_ = false;
	pause_state_ = false;
	alarm_state_ = false;
	alarm_exclusion_ = false;
	alarm_state_ack_ = false;
	moveInProgress_ = false;
	moveStopped_ = false;

	interpolation_data_.resize(3);
	first_time_ = true;
	pkg_path_ = ros::package::getPath("edo_core_pkg");
	jointCalib_ = 0;
	joints_number_ = 0;
	joints_mask_ = 0;
	joints_aux_mask_ = 0;
	configurationFileLoaded_ = 0;  // Configuration file not loaded yet
	aggPosCartPose_ = 0;
	control_mutex_ = PTHREAD_MUTEX_INITIALIZER;
	noCartPose_ = true;
	previousMoveFlyHandle_ = 0;
	idleMoveHandle_ = 0;
	
	bc_flag = false;
	CollisionFactor = 1;
	curr_limit[0] = 0.5;
	curr_limit[1] = 1.0;
	curr_limit[2] = 0.8;
	curr_limit[3] = 0.4;
    curr_limit[4] = 0.5;
	curr_limit[5] = 0.3;
	curr_limit[6] = 0.3; // set the threshold for the difference between current residuals for the double check
    
    _ORL_error = 1;
	_flag_set = true;
	
	private_nh.param<double>("controller_frequency", controller_frequency_, CONTROLLER_FREQUENCY);
	private_nh.param<double>("state_saturation_threshold", state_saturation_threshold_, 0.5);
	private_nh.param<int>("interpolation_time_step", interpolation_time_step_, INTERPOLATION_STEP);

	// Duration, callback, callback-owner, oneshot, autostart
	timerCalib_ = private_nh.createTimer(ros::Duration(0.5), &AlgorithmManager::timerCallback, this, true, false);
	collTimer   = private_nh.createTimer(ros::Duration(11), &AlgorithmManager::unbrakeTimerCallback, this, true, false);
	_collision_disable = false;
	
	ros::Rate loop_rate(controller_frequency_);
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
	last_publish_time = ros::Time::now();
#endif
	//Spin asincrono
	ros::AsyncSpinner aspin(2);
	aspin.start();

	while (ros::ok())
	{
		if (algorithm_mode_ != SWITCHED_OFF)
		{
			//update the control command
			updateControl();
			//publish the last control commands
			robot_control_publisher.publish(getCurrentControl());
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
			elapsed_publish_time = ros::Time::now() - last_publish_time;
			elapsed_time_in_msec = ((double)elapsed_publish_time.toNSec()) / 1000000.0f;  
			if (elapsed_time_in_msec > (5 * CONTROLLER_FREQUENCY))
						printf("[AlgorithmManager,%d] Elapsed time since the last publish %f\n", __LINE__, elapsed_time_in_msec);
			last_publish_time = ros::Time::now();
#endif
		}
		loop_rate.sleep();
	}

}

/**
 * Destructor of AlgorithmManager
 * - releases ORL library
 */
AlgorithmManager::~AlgorithmManager()
{
#if ENABLE_ROS_INFO
	ROS_INFO("Terminate controller...");
#endif
	if ((algorithm_mode_ != UNINITIALIZED) && (configurationFileLoaded_ == 1))
	{
		ORL_cancel_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		ORL_terminate_controller(LOCAL_VERBOSITY, ORL_CNTRL01);
	}
}

void AlgorithmManager::setAlgorithmMode(AlgorithmManager::Mode si_mode, const char *apc_func, int si_line)
{
	static const char *sam_AlgorithmMode[MAX_NUM_ALGORITHM_MODES] = 
	{
		"UNINITIALIZED(0)",
		"INITIALIZED(1)",
		"MOVING(2)",
		"WAITING(3)",
		"BLOCKED(4)",
		"FINISHED(5)",
		"PAUSE(6)",
		"RECOVERY(7)",
		"HOLD(8)",
		"SWITCHED_OFF(9)"
    };
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
  if (algorithm_mode_ != si_mode)
    printf("setAlgorithmMode [%s,%d] from <%s> to <%s>\n",apc_func,si_line,sam_AlgorithmMode[algorithm_mode_],sam_AlgorithmMode[si_mode]);
#endif
  algorithm_mode_ = si_mode;
  return;
}

bool AlgorithmManager::initializeORL()
{
	int status = ORL_initialize_controller("robot.edo", (pkg_path_ + "/config/").c_str(), LOCAL_VERBOSITY, ORL_CNTRL01);
	if(status < RET_OK)
	{
		ROS_ERROR("<ORL_initialize_controller> ORL error: %d", status);
		return false;
	}
	status = ORL_set_interpolation_time(interpolation_time_step_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	if(status < RET_OK)
	{
		ROS_ERROR("<ORL_set_interpolation_time> ORL error: %d", status);
		return false;
	}
	status = ORL_select_point_accuracy(ORL_TOL_NOSETTLE, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	if(status < RET_OK)
	{
		ROS_ERROR("<ORL_select_point_accuracy> ORL error: %d", status);
		return false;
	}
	{
		ORL_System_Variable orl_sys_var;
		orl_sys_var.ctype = ORL_STRING;
		ORL_get_SW_version(orl_sys_var.sysvar_name, LOCAL_VERBOSITY, ORL_CNTRL01); 
		ROS_INFO("SOFTWARE VERSION <%s>",orl_sys_var.sysvar_name);
	}
	base_.unit_type = ORL_CART_POSITION;
	base_.x = base_.y = base_.z = base_.a = base_.e = base_.r = 0.0f;
	tool_.unit_type = ORL_CART_POSITION;
	tool_.x = tool_.y = tool_.z = tool_.a = tool_.e = tool_.r = 0.0f;
	uframe_.unit_type = ORL_CART_POSITION;
	uframe_.x = uframe_.y = uframe_.z = uframe_.a = uframe_.e = uframe_.r = 0.0f;
	numberSteps_ = 0;
	numberSteps_M42_ = 0;
	return true;
}

bool AlgorithmManager::loadConfigurationFile_CB(edo_core_msgs::LoadConfigurationFile::Request  &req, edo_core_msgs::LoadConfigurationFile::Response &res){
  bool sb_sts = false;

	// printf("[loadConfigurationFile_CB,%d]\n",__LINE__);
	if ((algorithm_mode_ == UNINITIALIZED) && (configurationFileLoaded_ == 0)) // Configuration file already loaded?
	{ /* No */
		sb_sts = AlgorithmManager::initializeORL();
		if (sb_sts == false)
		{
			ROS_ERROR("Failure loading configuration file");
			//printf("[loadConfigurationFile_CB,%d] ROS_ERROR Failure loading configuration file\n",__LINE__);
		}
		else 
		{	
			configurationFileLoaded_ = 1;  // Configuration file loaded

			edo_core_msgs::JointsNumber::Request reqgjn;
			edo_core_msgs::JointsNumber::Response resgjn;

			joints_number_ = 0;

			if(!getSrvJointsNumber(reqgjn, resgjn))
			{
				ROS_ERROR("Impossible to get joints number");
			}
			else
			{
				joints_number_   = resgjn.counter;
				joints_mask_     = resgjn.joints_mask;
				joints_aux_mask_ = resgjn.joints_aux_mask;
	#if ENABLE_ROS_INFO
				ROS_INFO("ORL controller initialized...for %d axes",joints_number_);
				ROS_INFO("ORL controller initialized...joint mask %d",joints_mask_);
				ROS_INFO("ORL controller initialized...aux_mask   %d",joints_aux_mask_);
	#endif
				get_Strk(strk_);
			}
			noCartPose_ = false;  // Enable the execution of the kinematics
		}	
	}
#if DEVELOPMENT_RELEASE
	else
	{
		printf("[loadConfigurationFile_CB,%d] algorithm_mode_ %d configurationFileLoaded_ %d\n",__LINE__, algorithm_mode_, configurationFileLoaded_);
	}
#endif
	res.result = sb_sts;
	return(sb_sts);
}

bool AlgorithmManager::getSrvJointsNumber(edo_core_msgs::JointsNumber::Request  &req, edo_core_msgs::JointsNumber::Response &res){

	// printf("[getSrvJointsNumber, %d]\n",__LINE__);
	if (configurationFileLoaded_ == 1)  // Configuration file loaded
	{
		ORL_System_Variable     orl_sys_var;

		// $ARM_DATA[1].RRS_TOL_CTIME := 0
		memset(&orl_sys_var, 0, sizeof(ORL_System_Variable));
		sprintf(orl_sys_var.sysvar_name, "$ARM_DATA[%d].RRS_TOL_CTIME", ORL_ARM1 + 1); 
		orl_sys_var.ctype = ORL_INT; //  ORL_INT ORL_BOOL ORL_REAL ORL_STRING
		orl_sys_var.iv = 0;
		ORL_set_data   (orl_sys_var,      /* [IN]      Structure for the system variable */
        LOCAL_VERBOSITY , /* [IN]      Verbose ON/OFF */
        ORL_CNTRL01       /* [IN]      Controller Index */);
		
		// $ARM_DATA[1].RRS_TOL_FTIME := 0
		memset(&orl_sys_var, 0, sizeof(ORL_System_Variable));
		sprintf(orl_sys_var.sysvar_name, "$ARM_DATA[%d].RRS_TOL_FTIME", ORL_ARM1 + 1); 
		orl_sys_var.ctype = ORL_INT; //  ORL_INT ORL_BOOL ORL_REAL ORL_STRING
		orl_sys_var.iv = 0;
		ORL_set_data   (orl_sys_var,      /* [IN]      Structure for the system variable */
        LOCAL_VERBOSITY , /* [IN]      Verbose ON/OFF */
        ORL_CNTRL01       /* [IN]      Controller Index */);
		
		// $ARM_DATA[1].SING_CARE := TRUE
		memset(&orl_sys_var, 0, sizeof(ORL_System_Variable));
		sprintf(orl_sys_var.sysvar_name, "$ARM_DATA[%d].SING_CARE", ORL_ARM1 + 1); 
		orl_sys_var.ctype = ORL_BOOL; //  ORL_INT ORL_BOOL ORL_REAL ORL_STRING
		orl_sys_var.iv = ORL_FALSE;
		ORL_set_data   (orl_sys_var,      /* [IN]      Structure for the system variable */
        LOCAL_VERBOSITY , /* [IN]      Verbose ON/OFF */
        ORL_CNTRL01       /* [IN]      Controller Index */);
                    
		sprintf(orl_sys_var.sysvar_name, "$ARM_DATA[%d].AUX_MASK", ORL_ARM1 + 1);
		orl_sys_var.ctype = ORL_INT; //  ORL_BOOL  ORL_REAL  ORL_STRING
		int status_aux = ORL_get_data(&orl_sys_var, LOCAL_VERBOSITY, ORL_CNTRL01);
		int aux_mask = orl_sys_var.iv & SSM_JOINTS_MASK;
		if(!manageORLStatus(status_aux, "ORL_get_sys_var.$AUX_MASK"))
		{
			res.counter = -1;
#if DEVELOPMENT_RELEASE
			printf("[getSrvJointsNumber, %d] ORL_get_sys_var.$AUX_MASK failure\n",__LINE__);
#endif
			return true;
		}
		sprintf(orl_sys_var.sysvar_name, "$ARM_DATA[%d].JNT_MASK", ORL_ARM1 + 1);
		orl_sys_var.ctype = ORL_INT; //  ORL_BOOL  ORL_REAL  ORL_STRING
		int status_jnt = ORL_get_data(&orl_sys_var, LOCAL_VERBOSITY, ORL_CNTRL01);
		int jnt_mask = orl_sys_var.iv;
		int jnt_num;
		int jnt_mask_save;
		if(!manageORLStatus(status_jnt, "ORL_get_sys_var.$JNT_MASK"))
		{
			res.counter = -1;
#if DEVELOPMENT_RELEASE
			printf("[getSrvJointsNumber, %d] ORL_get_sys_var.$JNT_MASK failure\n",__LINE__);
#endif
			return true;
		}
		if (jnt_mask <= 0)
		{
			res.counter = -3;
#if DEVELOPMENT_RELEASE
			printf("[getSrvJointsNumber, %d] jnt_mask <= 0 \n",__LINE__);
#endif
			return true;
		}
		jnt_mask &= SSM_JOINTS_MASK; // safety check
		jnt_mask_save = jnt_mask;
		
		for(jnt_num = 0; jnt_mask != 0; jnt_mask >>= 1)
		{
			if (jnt_mask & 1)
				jnt_num++;
		}
    
		if (jnt_num > SSM_NUM_MAX_JOINTS)
			jnt_num = SSM_NUM_MAX_JOINTS;
    
#if DEVELOPMENT_RELEASE
		if ((jnt_num != 4) && (jnt_num != 6) && (jnt_num != 7))
			printf("[getSrvJointsNumber, %d] Jnt_Mask:%d Aux_Mask:%d Axes:%d\n",__LINE__, jnt_mask_save, aux_mask, jnt_num);
#endif

		res.counter = jnt_num;
		res.joints_mask = (unsigned long)jnt_mask_save;
		res.joints_aux_mask = (unsigned long)aux_mask;
    // printf("[getSrvJointsNumber, %d] Jnt_Mask:%d Aux_Mask:%d Axes:%d\n",__LINE__, jnt_mask_save, aux_mask, jnt_num);
	}
	else
	{
		res.counter = -2;
		// printf("[getSrvJointsNumber, %d] configuration not done!\n",__LINE__);
	}
	return true;
}

#if ENABLE_KINEMATICS_SERVICES
bool AlgorithmManager::getDirectKinematics(edo_core_msgs::DirectKinematics::Request  &req, edo_core_msgs::DirectKinematics::Response &res)
{
	ORL_joint_value joint_value;
	joint_value.unit_type = ORL_POSITION_LINK_DEGREE;
	
	for (size_t i = 0; i < req.positions.positions.size(); i++) 
	{
		joint_value.value[i] = req.positions.positions[i];
	}
	res.cartesian_pose = computeCartesianPose(&joint_value);
	return true;
}

bool AlgorithmManager::getInverseKinematics(edo_core_msgs::InverseKinematics::Request  &req, edo_core_msgs::InverseKinematics::Response &res)
{
	res.positions = computeJointValue(req.cartesian_pose);
	return true;
}
#endif

/**
 */
void AlgorithmManager::calibCallback(edo_core_msgs::JointCalibrationConstPtr msg)
{
  uint32_t sm_joints_mask = joints_mask_ & (uint32_t)msg->joints_mask;
	jointCalib_ |= sm_joints_mask;
	timerCalib_.stop();
	timerCalib_.start();
  return;
}


void AlgorithmManager::timerCallback(const ros::TimerEvent& event)
{
#if ENABLE_ROS_WARN
	ROS_WARN("TIMER CALLBACK. ZEROING AXES");
#endif	
	uint32_t sm_joints_mask = jointCalib_ & joints_mask_;
	jointCalib_ = 0;
	if (sm_joints_mask != 0)
	{
		for ( int i = 0; sm_joints_mask != 0; sm_joints_mask >>= 1, i++)
		{	
			if (sm_joints_mask & 1)
				hold_position_.value[i]=0.0;
		}
		first_time_ = true;  // So the first move will be done setting the initial position
	}

	return;
}

/**
 * This function handles the movement message received from state machine node
 * - a movement command can not be executed until a valid jnt_state has been
 *   received from the recovery node
 */
void AlgorithmManager::moveCallback(edo_core_msgs::MovementCommandConstPtr msg)
{
  int inLoop = 0;
  
  if(algorithm_mode_!=UNINITIALIZED)
  {
//    feedbackFn(MESSAGE_FEEDBACK::COMMAND_RECEIVED, 0, __FUNCTION__, __LINE__);

#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
printf ("[moveCallback,%d] Rx MOVE_MESSAGE_COMMAND = %c algorithm_mode_ %d\n",__LINE__,msg->move_command, algorithm_mode_);
#endif

#if ENABLE_ROS_INFO
    ROS_INFO("MOVE_MESSAGE_COMMAND = %c command received...   sono nello stato %d", msg->move_command, algorithm_mode_);
#endif

#if 0    
#############################################################################################################
    //gestisco il delay, settato al controllo precedente
    //se 255 passo oltre (no_delay), altrimenti aspetto
    if (pause_state_                                                 &&
        (msg->move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_RESUME) &&
        (msg->move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL) &&
        (msg->move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE))
    {
      ROS_ERROR("Edo is in pause state! Please resume or cancel previous movement...");
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
printf ("[moveCallback,%d] Edo is in pause state! Please resume or cancel previous move...\n",__LINE__);
#endif
      return;
    }
#############################################################################################################
#endif

#if 0    
#############################################################################################################
    if ((msg->move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_RESUME) &&
        (msg->move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL) &&
        (msg->move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE))
    {
      ros::Rate loop_rate(controller_frequency_);
      while((algorithm_mode_ == WAITING) || waiting_ )
      {
        if (++inLoop > 5000) {
	      #if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
          printf("[%s,%d] waiting in loop %d %d \n",__FUNCTION__,__LINE__,algorithm_mode_,waiting_);
		  #endif
          inLoop = 0;
        }
        if (!ros::ok())
        {
          return;
        }
        loop_rate.sleep();
      }
      //qui leggo il delay
      delay_ = msg->delay;
      if (msg->delay != 255 && msg->delay != 0) 
      {
        waiting_ = true;
      }
    }
#############################################################################################################
#endif

//PUSH IN CODA DEI MESSAGGI ARRIVATI, in testa vengono inseriti i messaggio più prioritari (PAUSE,RESUME,CANCEL) in coda i comandi di movimento
//la coda viene svuotata dal loop che genera i target

    switch (msg->move_command) 
    {
      case E_MOVE_COMMAND::E_MOVE_COMMAND_MOVE:
        msglist_.push_front(*msg);
      break;

      case E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE:
      case E_MOVE_COMMAND::E_MOVE_COMMAND_RESUME:
      case E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL:
        msglist_.push_back(*msg);
      break;
      
      default:
        ROS_WARN("Command not implemented, please use only accepted command type");
        feedbackFn(MESSAGE_FEEDBACK::ERROR, State::MOVETYPE_UNDEF, __FUNCTION__, __LINE__);
        return;
      break;
    }
  }
  else
  {
#if ENABLE_ROS_INFO
    ROS_INFO("Request for movement arrive while module was uninitialized");
#endif
    feedbackFn(MESSAGE_FEEDBACK::ERROR, State::ORL_UNINITIALIZED, __FUNCTION__, __LINE__);
  }

}


void AlgorithmManager::jogCallback(edo_core_msgs::MovementCommandConstPtr msg){

	if(algorithm_mode_!=UNINITIALIZED)
	{
//		feedbackFn(MESSAGE_FEEDBACK::COMMAND_RECEIVED, 0, __FUNCTION__, __LINE__);
#if ENABLE_ROS_INFO
		ROS_INFO("MOVE_MESSAGE_COMMAND = %c command received...", msg->move_command);
#endif
		switch (msg->move_command) 
		{
			case E_MOVE_COMMAND::E_MOVE_COMMAND_JOGMOVE:
				msglistJog_.push_front(*msg);
			break;

			case E_MOVE_COMMAND::E_MOVE_COMMAND_JOGSTOP:
				msglistJog_.push_back(*msg);
			break;

			default:
				ROS_WARN("Command not implemented, please use only accepted move commands");
				feedbackFn(MESSAGE_FEEDBACK::ERROR, State::MOVETYPE_UNDEF, __FUNCTION__, __LINE__);
				return;
			break;
		}
		
	}
	else
	{
#if ENABLE_ROS_INFO
		ROS_INFO("Request for movement arrive while module was uninitialized");
#endif
		feedbackFn(MESSAGE_FEEDBACK::ERROR, State::ORL_UNINITIALIZED, __FUNCTION__, __LINE__);
	}


}

/**
 * This function handles state message from recovery node
 * - the size of the control message is defined basing on the first state message
 *   @Warning this implies that once the joints has sent the first valid state
 *   the number of joints is fixed until the next startup
 */

void AlgorithmManager::stateCallback(edo_core_msgs::JointStateArrayConstPtr msg)
{
	bool JointsInAllarm = false;
	bool JointsInColl = false;
	bool JointsCurrentCalibration = false;
	unsigned long sm_JointsInBrakeState = 0;
	unsigned long sm_JointsInOverCurrentState = 0;
	uint8_t raspberry_coll_mask = 0;
	ORL_joint_value current_state_local;
	edo_core_msgs::CollisionAlgoToState coll_msg;
	
	for(size_t i = 0; i < msg->joints.size(); i++)
	{
	  _cur_res_joint[i] = msg->joints[i].R_jnt;
	}
		
	if (msg->joints.size() != joints_number_) {
		ROS_WARN_THROTTLE(10, "Unacceptable state, impossible to initialize ORL...");
		return;
	}
	memset(&current_state_local, 0, sizeof(ORL_joint_value));

	current_state_local.unit_type = ORL_POSITION_LINK_DEGREE;
	
#if 1
	if(!JointsCurrentCalibration)
	{
		if(msg->joints.size() >= (SSM_NUM_MAX_JOINTS - 1))
		{
			if((msg->joints[5].commandFlag & (1 << COMMAND_FLAG::UNCALIBRATED)) == 0)
			{
				JointsCurrentCalibration = true;
			}  
		}
	}
#endif

	for(size_t i = 0; i < msg->joints.size(); i++)
	{
      // If there are no errors, skip all the tests
      if ((msg->joints[i].commandFlag & 0xF0) != 0) // COMMAND_FLAG::STATUS_ERROR_MASK) == 0)
      {		

#if 1
		  if((msg->joints[i].commandFlag & (1 << COMMAND_FLAG::COLLISION)) != 0)
		  {
			  JointsInAllarm = true; 
			  sm_JointsInBrakeState |= (1 << i);
		  }
#endif
#if 1
		  if((msg->joints[i].commandFlag & (1 << COMMAND_FLAG::OVERCURRENT)) != 0)
		  {
			  JointsInAllarm = true;
              sm_JointsInOverCurrentState |= (1 << i); 
		  }
#endif
#if 1
		  if((msg->joints[i].commandFlag & (1 << COMMAND_FLAG::H_BRIDGE_DOWN)) != 0)
		  {
			  JointsInAllarm = true;
			  sm_JointsInBrakeState |= (1 << i);
		  }
#endif
		}
#if 1
		
		if(JointsCurrentCalibration && algorithm_mode_ != UNINITIALIZED && i<6) // se minore di 6, pinza non ha collision detection
		{
			
			if(collisionCheck( msg->joints[i].current, joints_control_.joints[i].current, i, curr_limit))
			{
				if(!_collision_disable)
				{
					JointsInColl = true;
					if(!bc_flag)
					{
						JointsInAllarm = true;
					}					
					sm_JointsInBrakeState |= (1 << i);
					raspberry_coll_mask |= (1<<i);
				}
			}
		}

#endif
		current_state_local.value[i] = msg->joints[i].position;
	}
	
	
  
	// Send collision message to State Machine
	coll_msg.coll_flag = JointsInColl;
	coll_msg.joints_mask = raspberry_coll_mask;
	algo_collision_publisher.publish(coll_msg);

	// Update the current state of the jnt position only if there are no errors
	if ((sm_JointsInOverCurrentState == 0) && (sm_JointsInBrakeState == 0))
	{
		memcpy(&current_state_, &current_state_local, sizeof(ORL_joint_value));
	}

	if (algorithm_mode_ == UNINITIALIZED)
	{ 
		// Configuration file already loaded?
		if (configurationFileLoaded_ == 1)
		{
			// Yes, do the initialization
			// The very first time, when the robot is switched on, the status is 'brake' since the beginnig.
			// so the current position of the robot is not set.
			// Here we force the setting, just before the first 'unbrake' of the system
			memcpy(&current_state_, &current_state_local, sizeof(ORL_joint_value));
			initialize(msg);
		}
	}
	else if (algorithm_mode_ != SWITCHED_OFF)
	{
		if (++aggPosCartPose_ >= 10)
		{
			// every ~100ms compute the cartesian position
			if (alarm_state_ == false)
			{
				cartesian_pose_pub_.publish(computeCartesianPose(&current_state_));
				aggPosCartPose_ = 0;
			}
			else
			{
				--aggPosCartPose_;  // As soon as there is no more the alarm, redo the direct kinematics.
			}
		}

		if (JointsInAllarm == true)
		{
			if (alarm_state_ == false)
			{
				if (moveInProgress_ == true)
				{
					(void)ORL_stop_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		               #if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
					printf("[stateCallback,%d] ORL_stop_motion\n",__LINE__);
		               #endif  
				}
				else
				/* If the move is already ended we have to handle the alarm here */
				{
					dynamicModelReset();
				}
				alarm_state_ = true;
			}
		}
		else
		{
			if ((moveInProgress_ == false) || (alarm_state_ack_ == true))
			{
				alarm_state_ = false;
				alarm_state_ack_ = false;
			}
		}
		
	}
	return;
}

static void frame_dump(const char *name, int line, ORL_cartesian_position cp);
static void frame_dump(const char *name, int line, ORL_cartesian_position cp)
{
#if 0 // ENABLE_ROS_INFO
	ROS_INFO("[%s,%d] ", name, line, cp.x);
	ROS_INFO("[%s,%d] ", name, line, cp.y);
	ROS_INFO("[%s,%d] ", name, line, cp.z);
#endif
	return;
}

/**
 * 
 */

bool AlgorithmManager::setReferenceFn(edo_core_msgs::MovementCommand * msg)
{
  int status = 0;

  {

    ORL_cartesian_position tool_local;
    ORL_cartesian_position uframe_local;

    memset(&tool_local, 0, sizeof(ORL_cartesian_position));
    tool_local.unit_type = ORL_CART_POSITION;
    tool_local.x = msg->tool.x;
    tool_local.y = msg->tool.y;
    tool_local.z = msg->tool.z;
    tool_local.a = msg->tool.a; 
    tool_local.e = msg->tool.e;
    tool_local.r = msg->tool.r;
    frame_dump("TOOL", __LINE__, tool_local);
    uframe_local.unit_type = ORL_CART_POSITION;
    uframe_local.x = msg->frame.x;
    uframe_local.y = msg->frame.y;
    uframe_local.z = msg->frame.z;
    uframe_local.a = msg->frame.a;
    uframe_local.e = msg->frame.e;
    uframe_local.r = msg->frame.r;
    frame_dump("UFRAME", __LINE__, uframe_local);
    /*                             $BASE     $TOOL       $UFRAME        */

    status = ORL_initialize_frames (base_, tool_local, uframe_local, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);

    if(!manageORLStatus(status,"ORL_initialize_frames"))
    {
      return false;
    }
  }

#if 0 // just for testing
  { 
    ORL_payload_value  payload_data;  

    memset(&payload_data, 0, sizeof(ORL_payload_value));

    payload_data.tool_mass       = msg->tool.a; // Just for test see above
    payload_data.tool_cntr_x     = 0;
    payload_data.tool_cntr_y     = 0;
    payload_data.tool_cntr_z     = 0;
    payload_data.tool_inertia_xx = 0;
    payload_data.tool_inertia_yy = 0;
    payload_data.tool_inertia_zz = 0;
    payload_data.tool_inertia_xy = 0;
    payload_data.tool_inertia_xz = 0;
    payload_data.tool_inertia_yz = 0;

	status = ORL_initialize_payload (payload_data, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);

    if(!manageORLStatus(status,"ORL_initialize_payload"))
    {
      return false;
    }
  }
#endif

#if 0
  {
    ORL_payload_value  payload_data;  

    memset(&payload_data, 0, sizeof(ORL_payload_value));

    payload_data.tool_mass       = msg->payload.mass; // Just for test see above
    payload_data.tool_cntr_x     = msg->payload.x;
    payload_data.tool_cntr_y     = msg->payload.y;
    payload_data.tool_cntr_z     = msg->payload.z;
    payload_data.tool_inertia_xx = msg->payload.xx;
    payload_data.tool_inertia_yy = msg->payload.yy;
    payload_data.tool_inertia_zz = msg->payload.zz;
    payload_data.tool_inertia_xy = msg->payload.xy;
    payload_data.tool_inertia_xz = msg->payload.xz;
    payload_data.tool_inertia_yz = msg->payload.yz;

	status = ORL_initialize_payload (payload_data, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);

    if(!manageORLStatus(status,"ORL_initialize_payload"))
    {
      return false;
    }
  }
#endif
    return true;
}

/**
 * This function is called only once during the life of an instance
 * @param msg the first message received from /jnt_state
 * - sets the size of the joints vector inside the joints_control message
 *   according to the joints number in msg->size
 * - sets current control value to the value received from /jnt_state
 * - changes the state of the algorithm instance to AlgorithmManager::Mode::INITIALIZED
 */

//
// MOVE JOINT TO <>
//
bool AlgorithmManager::moveTrjntFn(edo_core_msgs::MovementCommand * msg)
{
  int status = 0;
  unsigned int sj_lastJointAxis;
  unsigned long sm_joints_mask;

#if ENABLE_ROS_INFO
  ROS_INFO("[%d] Move_command:%c move_type:%c", __LINE__, msg->move_command, msg->move_type);
  ROS_INFO("[%d] Target.data_type:%c", __LINE__, msg->target.data_type);
#endif

  if (msg->move_type != E_MOVE_TYPE::E_MOVE_TYPE_JOINT)
  {
    ROS_ERROR("[%d]Invalid Joint MOVE_TYPE:%c", __LINE__, msg->move_type);
    return false;
  }
  if ((msg->target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_JOINT)    &&
      (msg->target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION) &&
      (msg->target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_XTND_POS))
  {
    ROS_ERROR("[%d]Invalid Destination Data Type:%c", __LINE__, msg->target.data_type);
    return false;
  }

  if (msg->target.data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_JOINT)
  {
    // MOVE JOINT TO <joint_position>
    target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
    sm_joints_mask = joints_mask_ & (unsigned long)msg->target.joints_mask;
    sj_lastJointAxis = CommonService::getLastJointAxis(sm_joints_mask);
    for(unsigned int i = 0; i < sj_lastJointAxis; i++)
    {
	  
      if (sm_joints_mask & (1 << i))
      {

	    // gestione stroke ends superabili
		if(msg->target.joints_data[i]>strk_[IDX_STRK_ENP_P][i] && msg->target.joints_data[i]<strk_[IDX_STRK_ENP_P][i]+0.2)
		{
		  msg->target.joints_data[i] = strk_[IDX_STRK_ENP_P][i];
		}
		else if (msg->target.joints_data[i]<strk_[IDX_STRK_ENP_N][i] && msg->target.joints_data[i]>strk_[IDX_STRK_ENP_N][i]-0.2)
		{
	      msg->target.joints_data[i] = strk_[IDX_STRK_ENP_N][i];
		}
		
        target_joint_.value[i] = msg->target.joints_data[i];
// printf("[moveTrjntFn,%d] Joint %d moving from %f to %f\n",__LINE__,i,current_state_.value[i],target_joint_.value[i]);
#if ENABLE_ROS_INFO
        ROS_INFO("Received data %d is : %f", i + 1,  target_joint_.value[i]);
#endif
      }
      else
      {
        target_joint_.value[i] = current_state_.value[i];
      }
    }
  }
  else if (msg->target.data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION)
  {
    // MOVE JOINT TO <cartesian_position>
    //memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
    target_cart_.unit_type = ORL_CART_POSITION;
    target_cart_.x = msg->target.cartesian_data.x;
    target_cart_.y = msg->target.cartesian_data.y;
    target_cart_.z = msg->target.cartesian_data.z;
    target_cart_.a = msg->target.cartesian_data.a;
    target_cart_.e = msg->target.cartesian_data.e;
    target_cart_.r = msg->target.cartesian_data.r;

    if (msg->target.cartesian_data.config_flags.length() > 0) {
      strncpy(target_cart_.config_flags, msg->target.cartesian_data.config_flags.c_str(), SSM_CONFIG_FLAG_STRLEN_MAX);
    }
#if ENABLE_ROS_INFO
    ROS_INFO("Received Cartesian x: %f, y: %f, z: %f, a: %f, e: %f, r: %f",target_cart_.x, target_cart_.y, target_cart_.z, target_cart_.a, target_cart_.e, target_cart_.r);
    ROS_INFO("tags: <%s>", target_cart_.config_flags);
#endif
  }
  else
  { 
    // MOVE JOINT TO <xtnd_position>
    target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;

    sm_joints_mask = joints_mask_ & (unsigned long)msg->target.joints_mask;
    sj_lastJointAxis = CommonService::getLastJointAxis(sm_joints_mask);
#if ENABLE_ROS_INFO
//    ROS_INFO("Last joint axis is %d", sj_lastJointAxis);
#endif
    for(unsigned int i = 0; i < sj_lastJointAxis; i++)
    {
      if (sm_joints_mask & (1 << i))
      {
        target_joint_.value[i] = msg->target.joints_data[i];
#if ENABLE_ROS_INFO
        ROS_INFO("Received data %d is : %f", i + 1,  target_joint_.value[i]);
#endif
      }
      else
      {
        target_joint_.value[i] = current_state_.value[i];
      }
    }
    //memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
    target_cart_.unit_type = ORL_CART_POSITION;
    target_cart_.x = msg->target.cartesian_data.x;
    target_cart_.y = msg->target.cartesian_data.y;
    target_cart_.z = msg->target.cartesian_data.z;
    target_cart_.a = msg->target.cartesian_data.a;
    target_cart_.e = msg->target.cartesian_data.e;
    target_cart_.r = msg->target.cartesian_data.r;

	if (msg->target.cartesian_data.config_flags.length() > 0) 
	{
		strncpy(target_cart_.config_flags, msg->target.cartesian_data.config_flags.c_str(), SSM_CONFIG_FLAG_STRLEN_MAX);
	}
#if ENABLE_ROS_INFO
    ROS_INFO("Received Cartesian x: %f, y: %f, z: %f, a: %f, e: %f, r: %f",target_cart_.x, target_cart_.y, target_cart_.z, target_cart_.a, target_cart_.e, target_cart_.r);
    ROS_INFO("tags: <%s>", target_cart_.config_flags);
#endif
  }

  if(first_time_)
  {
#if ENABLE_ROS_WARN
    ROS_WARN("set position");
#endif
    status = ORL_set_position(NULL, &current_state_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    if(!manageORLStatus(status,"ORL_set_position"))
    {
      return false;
    }
    int status_dyn;
    ORL_Dynamic_Model sx_dyn_mod_fake;
    memset(&sx_dyn_mod_fake, 0, sizeof(ORL_Dynamic_Model));
	
    for(size_t i = 0; i < NR_GET_NEXT_STEP_CALLS ; i++)
	{
      status_dyn = ORL_get_dynamic_model(NULL, NULL, NULL, &sx_dyn_mod_fake, 1, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    }
    
    first_time_ = false;
  }

  {
    if(!setReferenceFn(msg))
    {
      return false;
    }
	
  }
  
  {
    std::vector<int> move_parameters;
    move_parameters.resize(3);
    
    int status = 0;
    if (msg->delay == 255) 
    {
      move_parameters[0] = ORL_FLY;
      move_parameters[1] = ORL_ADVANCE;
    }
    else
    {
      move_parameters[0] = ORL_NO_FLY;
      move_parameters[1] = ORL_WAIT;
    }
    move_parameters[2] = ORL_FLY_NORMAL;
  
    status = setORLMovement(move_parameters, msg->move_type, msg->target.data_type, msg->ovr);
    if(!manageORLStatus(status,"setORLMovement"))
    {
      return false;
    }
    MoveCommandMsgListInOrl_.push_back(*msg);  // Now ORL has this Move Command
#if ENABLE_ALGOMNGR_PRINTFS
    printf("[moveTrjntFn,%d] ORL Steps %d MCLS %d Fly %s Ax1:%f\n",__LINE__, numberSteps_, MoveCommandMsgListInOrl_.size(),(move_parameters[0] == ORL_FLY) ? "Yes" : "No", msg->target.joints_data[0]);
#endif
  }

  setAlgorithmMode(MOVING, __FUNCTION__,__LINE__);
  return true;
}

//
// MOVE LINEAR TO <>
//
bool AlgorithmManager::moveCarlinFn(edo_core_msgs::MovementCommand * msg)
{
  int status = 0;
  unsigned int sj_lastJointAxis;
  unsigned long sm_joints_mask;
  unsigned long sm_joints_aux_mask = joints_aux_mask_;

#if ENABLE_ROS_INFO
  ROS_INFO("[%d] Move_command:%c move_type:%c", __LINE__, msg->move_command, msg->move_type);
  ROS_INFO("[%d] Target.data_type:%c", __LINE__, msg->target.data_type);
#endif

  if (msg->move_type != E_MOVE_TYPE::E_MOVE_TYPE_LINEAR)
  {
    ROS_ERROR("[%d]Invalid Cartesian MOVE_TYPE:%c", __LINE__, msg->move_type);
    return false;
  }
  if ((msg->target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_JOINT)    &&
      (msg->target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION) &&
      (msg->target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_XTND_POS))
  {
    ROS_ERROR("[%d]Invalid Destination Data Type:%c", __LINE__, msg->target.data_type);
    return false;
  }

  if (msg->target.data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_JOINT)
  {// MOVE LINEAR TO <joint_position>
    target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
    sm_joints_mask = joints_mask_& (unsigned long)msg->target.joints_mask;
    sj_lastJointAxis = CommonService::getLastJointAxis(sm_joints_mask);
    for(unsigned int i = 0; i < sj_lastJointAxis; i++)
    {
      if (sm_joints_mask & (1 << i))
      {
        target_joint_.value[i] = msg->target.joints_data[i];
#if ENABLE_ROS_INFO
        ROS_INFO("Received data %d is : %f", i + 1,  target_joint_.value[i]);
#endif
      }
      else
      {
        target_joint_.value[i] = 0.0;
      }
    }
  }
  else if (msg->target.data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION)
  {// MOVE LINEAR TO <cartesian_position>
    //memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
    target_cart_.unit_type = ORL_CART_POSITION;
    target_cart_.x = msg->target.cartesian_data.x;
    target_cart_.y = msg->target.cartesian_data.y;
    target_cart_.z = msg->target.cartesian_data.z;
    target_cart_.a = msg->target.cartesian_data.a;
    target_cart_.e = msg->target.cartesian_data.e;
    target_cart_.r = msg->target.cartesian_data.r;

    if (msg->target.cartesian_data.config_flags.length() > 0) {
      strncpy(target_cart_.config_flags, msg->target.cartesian_data.config_flags.c_str(), SSM_CONFIG_FLAG_STRLEN_MAX);
    }
#if ENABLE_ROS_INFO
    ROS_INFO("Received Cartesian x: %f, y: %f, z: %f, a: %f, e: %f, r: %f",target_cart_.x, target_cart_.y, target_cart_.z, target_cart_.a, target_cart_.e, target_cart_.r);
    ROS_INFO("tags: <%s>", target_cart_.config_flags);
#endif
    if (sm_joints_aux_mask != 0)
    {
      target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
      sm_joints_mask = joints_mask_ & (unsigned long)msg->target.joints_mask;
      sj_lastJointAxis = CommonService::getLastJointAxis(sm_joints_mask);
      for(unsigned int i = 0; i < sj_lastJointAxis; i++)
      {
        if (sm_joints_aux_mask & (1 << i))
        {
          target_joint_.value[i] = current_state_.value[i];
        }
        else
        {
          target_joint_.value[i] = 0.0;
        }
      } 
      msg->target.data_type = E_MOVE_DEST_POINT::E_MOVE_POINT_XTND_POS; // Fake change of the destination position from 'position' to 'xtnd_position'
    }
  }
  else
  {
    // MOVE LINEAR TO <xtnd_position>
    target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;

    sm_joints_mask = joints_mask_ & (unsigned long)msg->target.joints_mask;
    sj_lastJointAxis = CommonService::getLastJointAxis(sm_joints_mask);
    for(unsigned int i = 0; i < sj_lastJointAxis; i++)
    {
      if (sm_joints_mask & (1 << i))
      {
        target_joint_.value[i] = msg->target.joints_data[i];
      }
      else
      {
        target_joint_.value[i] = 0.0;
      }
    }
    //memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
    target_cart_.unit_type = ORL_CART_POSITION;
    target_cart_.x = msg->target.cartesian_data.x;
    target_cart_.y = msg->target.cartesian_data.y;
    target_cart_.z = msg->target.cartesian_data.z;
    target_cart_.a = msg->target.cartesian_data.a;
    target_cart_.e = msg->target.cartesian_data.e;
    target_cart_.r = msg->target.cartesian_data.r;

    if (msg->target.cartesian_data.config_flags.length() > 0) {
      strncpy(target_cart_.config_flags, msg->target.cartesian_data.config_flags.c_str(), SSM_CONFIG_FLAG_STRLEN_MAX);
    }
#if ENABLE_ROS_INFO
    ROS_INFO("Received Cartesian x: %f, y: %f, z: %f, a: %f, e: %f, r: %f",target_cart_.x, target_cart_.y, target_cart_.z, target_cart_.a, target_cart_.e, target_cart_.r);
    ROS_INFO("tags: <%s>", target_cart_.config_flags);
#endif
  }

  if (first_time_)
  {
#if ENABLE_ROS_WARN
    ROS_WARN("set position");
#endif
    status = ORL_set_position(NULL, &current_state_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    if(!manageORLStatus(status,"ORL_set_position"))
    {
      return false;
    }
    int status_dyn;
    ORL_Dynamic_Model sx_dyn_mod_fake;
	memset(&sx_dyn_mod_fake, 0, sizeof(ORL_Dynamic_Model));

    for(size_t i = 0; i < NR_GET_NEXT_STEP_CALLS ; i++)
	{
      status_dyn = ORL_get_dynamic_model(NULL, NULL, NULL, &sx_dyn_mod_fake, 1, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    }

    first_time_ = false;
  }

  {

    if(!setReferenceFn(msg))
    {
      return false;
    }
  }

  {
    std::vector<int> move_parameters;
    move_parameters.resize(3);

    if (msg->delay == 255) {
      move_parameters[0] = ORL_FLY;
      move_parameters[1] = ORL_ADVANCE;
    }
    else
    {
      move_parameters[0] = ORL_NO_FLY;
      move_parameters[1] = ORL_WAIT;
    }

    move_parameters[2] = ORL_FLY_NORMAL;
    status = setORLMovement(move_parameters, msg->move_type, msg->target.data_type, msg->ovr);
    if(!manageORLStatus(status,"setORLMovement"))
    {
      return false;
    }
    MoveCommandMsgListInOrl_.push_back(*msg);  // Now ORL has this Move Command
#if ENABLE_ALGOMNGR_PRINTFS
    printf("[moveCarlinFn,%d] ORL Move Command List Size %d\n",__LINE__,MoveCommandMsgListInOrl_.size());
#endif
  }

  setAlgorithmMode(MOVING, __FUNCTION__,__LINE__);
  return true;
}

//
// MOVE CIRCULAR TO <>
//
bool AlgorithmManager::moveCarcirFn(edo_core_msgs::MovementCommand * msg)
{
  int status = 0;
  unsigned int sj_lastJointAxis;
  unsigned long sm_joints_mask;
  unsigned long sm_joints_aux_mask = joints_aux_mask_;

#if ENABLE_ROS_INFO
  ROS_INFO("[%d] Move_command:%c move_type:%c", __LINE__, msg->move_command, msg->move_type);
  ROS_INFO("[%d] Target.data_type:%c", __LINE__, msg->target.data_type);
  ROS_INFO("[%d] Via.data_type:%c", __LINE__, msg->via.data_type);
#endif

  if (msg->move_type != E_MOVE_TYPE::E_MOVE_TYPE_CIRCULAR)
  {
    ROS_ERROR("[%d] Invalid Cartesian Command:%c", __LINE__, msg->move_type);
    return false;
  }
  if ((msg->target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_JOINT)    &&
      (msg->target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION) &&
      (msg->target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_XTND_POS))
  {
    ROS_ERROR("[%d] Invalid Destination Data Type:%c", __LINE__, msg->target.data_type);
    return false;
  }
  if ((msg->via.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_JOINT)    &&
      (msg->via.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION) &&
      (msg->via.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_XTND_POS))
  {
    ROS_ERROR("[%d] Invalid Via Data Type:%c", __LINE__, msg->via.data_type);
    return false;
  }

  if (msg->target.data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_JOINT)
  {
    // MOVE CIRCULAR TO <joint_position> VIA <>
    target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
    sm_joints_mask = joints_mask_ & (unsigned long)msg->target.joints_mask;
    sj_lastJointAxis = CommonService::getLastJointAxis(sm_joints_mask);
    for(unsigned int i = 0; i < sj_lastJointAxis; i++)
    {
      if (sm_joints_mask & (1 << i))
      {
        target_joint_.value[i] = msg->target.joints_data[i];
#if ENABLE_ROS_INFO
        ROS_INFO("Received data %d is : %f", i + 1,  target_joint_.value[i]);
#endif
      }
      else
      {
        target_joint_.value[i] = 0.0;
      }
    }
  }
  else if (msg->target.data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION)
  {
    // MOVE CIRCULAR TO <cartesian_position> VIA <>
    //memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
    target_cart_.unit_type = ORL_CART_POSITION;
    target_cart_.x = msg->target.cartesian_data.x;
    target_cart_.y = msg->target.cartesian_data.y;
    target_cart_.z = msg->target.cartesian_data.z;
    target_cart_.a = msg->target.cartesian_data.a;
    target_cart_.e = msg->target.cartesian_data.e;
    target_cart_.r = msg->target.cartesian_data.r;

    if (msg->target.cartesian_data.config_flags.length() > 0) {
      strncpy(target_cart_.config_flags, msg->target.cartesian_data.config_flags.c_str(), SSM_CONFIG_FLAG_STRLEN_MAX);
    }
    
    if (sm_joints_aux_mask != 0)
    {
      target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
      
      sm_joints_mask = joints_mask_ & (unsigned long)msg->target.joints_mask;
      sj_lastJointAxis = CommonService::getLastJointAxis(sm_joints_mask);
      for(unsigned int i = 0; i < sj_lastJointAxis; i++)
      {
        if (sm_joints_aux_mask & (1 << i))
        {
          target_joint_.value[i] = current_state_.value[i];
        }
        else
        {
          target_joint_.value[i] = 0.0;
        }
      } 
      msg->target.data_type = E_MOVE_DEST_POINT::E_MOVE_POINT_XTND_POS; // Fake change of the destination position from 'position' to 'xtnd_position'
    }
  }
  else 
  {
    // MOVE CIRCULAR TO <xtnd_position> VIA <>
    target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
    
    sm_joints_mask = joints_mask_ & (unsigned long)msg->target.joints_mask;
    sj_lastJointAxis = CommonService::getLastJointAxis(sm_joints_mask);
    for(unsigned int i = 0; i < sj_lastJointAxis; i++)
    {
      if (sm_joints_mask & (1 << i))
      {
        target_joint_.value[i] = msg->target.joints_data[i];
      }
      else
      {
        target_joint_.value[i] = 0.0;
      }
    }
    //memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
    target_cart_.unit_type = ORL_CART_POSITION;
    target_cart_.x = msg->target.cartesian_data.x;
    target_cart_.y = msg->target.cartesian_data.y;
    target_cart_.z = msg->target.cartesian_data.z;
    target_cart_.a = msg->target.cartesian_data.a;
    target_cart_.e = msg->target.cartesian_data.e;
    target_cart_.r = msg->target.cartesian_data.r;
    
    if (msg->target.cartesian_data.config_flags.length() > 0) {
      strncpy(target_cart_.config_flags, msg->target.cartesian_data.config_flags.c_str(), SSM_CONFIG_FLAG_STRLEN_MAX);
    }
#if DEVELOPMENT_RELEASE
    ROS_INFO("Received Cartesian x: %f, y: %f, z: %f, a: %f, e: %f, r: %f",target_cart_.x, target_cart_.y, target_cart_.z, target_cart_.a, target_cart_.e, target_cart_.r);
    ROS_INFO("tags: <%s>", target_cart_.config_flags);
#endif
  }

  if (msg->via.data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_JOINT)
  {
    // MOVE CIRCULAR TO <> VIA <joint_position>
    target_joint_via_.unit_type = ORL_POSITION_LINK_DEGREE;
    sm_joints_mask = joints_mask_ & (unsigned long)msg->via.joints_mask;
    sj_lastJointAxis = CommonService::getLastJointAxis(sm_joints_mask);
    for(unsigned int i = 0; i < sj_lastJointAxis; i++)
    {
      if (sm_joints_mask & (1 << i))
      {
        target_joint_via_.value[i] = msg->via.joints_data[i];
#if ENABLE_ROS_INFO
        ROS_INFO("Received data %d is : %f", i + 1,  target_joint_via_.value[i]);
#endif
      }
      else
      {
        target_joint_via_.value[i] = 0.0;
      }
    }
  }
  else if (msg->via.data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION)
  {
    // MOVE CIRCULAR TO <> VIA <cartesian_position>
    memset(&target_cart_via_, 0, sizeof(ORL_cartesian_position));
    target_cart_via_.unit_type = ORL_CART_POSITION;
    target_cart_via_.x = msg->via.cartesian_data.x;
    target_cart_via_.y = msg->via.cartesian_data.y;
    target_cart_via_.z = msg->via.cartesian_data.z;
    target_cart_via_.a = msg->via.cartesian_data.a;
    target_cart_via_.e = msg->via.cartesian_data.e;
    target_cart_via_.r = msg->via.cartesian_data.r;

    if (msg->via.cartesian_data.config_flags.length() > 0) {
      strncpy(target_cart_via_.config_flags, msg->via.cartesian_data.config_flags.c_str(), SSM_CONFIG_FLAG_STRLEN_MAX);
    }
  }
  else 
  { 
    // MOVE CIRCULAR TO <> VIA <xtnd_position>
    target_joint_via_.unit_type = ORL_POSITION_LINK_DEGREE;
    sm_joints_mask = joints_mask_ & (unsigned long)msg->via.joints_mask;
    sj_lastJointAxis = CommonService::getLastJointAxis(sm_joints_mask);
    for(unsigned int i = 0; i < sj_lastJointAxis; i++)
    {
      if (sm_joints_mask & (1 << i))
      {
        target_joint_via_.value[i] = msg->via.joints_data[i];
#if ENABLE_ROS_INFO
        ROS_INFO("Received data %d is : %f", i + 1,  target_joint_via_.value[i]);
#endif
      }
      else
      {
        target_joint_via_.value[i] = 0.0;
      }
    }
    memset(&target_cart_via_, 0, sizeof(ORL_cartesian_position));
    target_cart_via_.unit_type = ORL_CART_POSITION;
    target_cart_via_.x = msg->via.cartesian_data.x;
    target_cart_via_.y = msg->via.cartesian_data.y;
    target_cart_via_.z = msg->via.cartesian_data.z;
    target_cart_via_.a = msg->via.cartesian_data.a;
    target_cart_via_.e = msg->via.cartesian_data.e;
    target_cart_via_.r = msg->via.cartesian_data.r;

    if (msg->via.cartesian_data.config_flags.length() > 0) {
      strncpy(target_cart_via_.config_flags, msg->via.cartesian_data.config_flags.c_str(), SSM_CONFIG_FLAG_STRLEN_MAX);
    }
  } 
  
  if(first_time_)
  {
    status = ORL_set_position(NULL, &current_state_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    if(!manageORLStatus(status,"ORL_set_position"))
    {
      return false;
    }
    int status_dyn;
    ORL_Dynamic_Model sx_dyn_mod_fake;
	memset(&sx_dyn_mod_fake, 0, sizeof(ORL_Dynamic_Model));
    
    for(size_t i = 0; i < NR_GET_NEXT_STEP_CALLS ; i++)
	{
      status_dyn = ORL_get_dynamic_model(NULL, NULL, NULL, &sx_dyn_mod_fake, 1, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    }
    first_time_ = false;
  }
  
  {

    if(!setReferenceFn(msg))
    {
      return false;
    }
  }

  {  
    std::vector<int> move_parameters;
    
    move_parameters.resize(4);
    if (msg->delay == 255) {
      move_parameters[0] = ORL_FLY;
      move_parameters[1] = ORL_ADVANCE;
    }
    else
    {
      move_parameters[0] = ORL_NO_FLY;
      move_parameters[1] = ORL_WAIT;
    }
    move_parameters[2] = ORL_FLY_NORMAL;
    
	move_parameters[3] = ORL_CIR_VIA;
    status = setORLMovement(move_parameters, msg->move_type, msg->via.data_type, msg->ovr);
    if(!manageORLStatus(status,"setORLMovement"))
    {
      return false;
    }

	move_parameters[3] = ORL_CIR_TARGET;
    status = setORLMovement(move_parameters, msg->move_type, msg->target.data_type, msg->ovr);
    if(!manageORLStatus(status,"setORLMovement"))
    {
      return false;
    }
    
    MoveCommandMsgListInOrl_.push_back(*msg);  // Now ORL has this Move Command
#if ENABLE_ALGOMNGR_PRINTFS
    printf("[moveCarcirFn,%d] ORL Move Command List Size %d\n",__LINE__,MoveCommandMsgListInOrl_.size());
#endif
  }
  
  setAlgorithmMode(MOVING, __FUNCTION__,__LINE__);
  return true;
}

//funzione di jog in spazio giunti
bool AlgorithmManager::jogTrjntFn(edo_core_msgs::MovementCommand *  msg)
{
  int status = 0;
    
  ORL_System_Variable Sysvar;
  ORL_joint_value current_state_local;
  double minimum=1.0;

#if ENABLE_ROS_INFO
  ROS_INFO("[%d] Move_command:%c move_type:%c", __LINE__, msg->move_command, msg->move_type);
  ROS_INFO("[%d] Target.data_type:%c", __LINE__, msg->target.data_type);
#endif
  
  if (msg->target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_JOINT)
  {
    return false;
  }
  
  current_state_local.unit_type = ORL_POSITION_LINK_DEGREE;
  for(size_t i = 0; i < ORL_MAX_AXIS; i++)
  {
    if(i == N_AX_GRIPPER)
      current_state_local.value[i]=std::max(strk_[IDX_STRK_ENP_N][i] + 0.2, std::min(strk_[IDX_STRK_ENP_P][i]-0.2, current_state_.value[i]));
    else
      current_state_local.value[i]=std::max(strk_[IDX_STRK_ENP_N][i] + 0.1, std::min(strk_[IDX_STRK_ENP_P][i]-0.1, current_state_.value[i]));
  }
  
  memcpy(&target_joint_, &current_state_local, sizeof(ORL_joint_value));
  target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
  {
    unsigned int sj_lastJointAxis;
    unsigned long sm_joints_mask;

#if ENABLE_ROS_WARN
    ROS_WARN("Joints Mask %lld", msg->target.joints_mask);
#endif
    sm_joints_mask = joints_mask_ & (unsigned long)msg->target.joints_mask;
    sj_lastJointAxis = CommonService::getLastJointAxis(sm_joints_mask);
#if ENABLE_ROS_WARN
    ROS_WARN("Last Joint Axis %d", sj_lastJointAxis);
#endif
#if ENABLE_ALGOMNGR_PRINTFS
printf("[%s,%d] Mask %d LastJointAxis %d\n",__FILE__,__LINE__,sm_joints_mask,sj_lastJointAxis);  
#endif
    for(unsigned int i = 0; i < sj_lastJointAxis; i++)
    {
      if (sm_joints_mask & (1 << i))
      {
        if(fabs(msg->target.joints_data[i]) > 0.01)
        {
          minimum=std::min(fabs(msg->target.joints_data[i]),minimum);
          if(i == N_AX_GRIPPER)
            target_joint_.value[i] = (msg->target.joints_data[i] < 0) ? strk_[IDX_STRK_ENP_N][i] + 0.2 : strk_[IDX_STRK_ENP_P][i] - 0.2;
          else
            target_joint_.value[i] = (msg->target.joints_data[i] < 0) ? strk_[IDX_STRK_ENP_N][i] + 0.1 : strk_[IDX_STRK_ENP_P][i] - 0.1;
// printf("[%s,%d] Joint %d has a delta %f\n",__FILE__,__LINE__,i,target_joint_.value[i]);
        }
#if ENABLE_ROS_INFO
        ROS_INFO("Set data %d is : %f", i + 1,  target_joint_.value[i]);
#endif
      }
      else
      {
        target_joint_.value[i] = 0.0;
      }
    }
  }

  if (algorithm_mode_ != MOVING)
  {
    status = ORL_set_position(NULL, &current_state_local, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    if(!manageORLStatus(status,"ORL_set_position"))
    {
      return false;
    }
  }
  
  {

    if(!setReferenceFn(msg))
    {
      return false;
    }
  }

  {
    std::vector<int> move_parameters;
    move_parameters.resize(3);

    move_parameters[0] = ORL_NO_FLY;
    move_parameters[1] = ORL_WAIT;
    move_parameters[2] = ORL_FLY_NORMAL;
    
    msg->move_command = E_MOVE_COMMAND::E_MOVE_COMMAND_JOGMOVE;
    msg->move_type = E_MOVE_TYPE::E_MOVE_TYPE_JOINT;
    jog_target_data_type_ = msg->target.data_type;
    status = setORLMovement(move_parameters, msg->move_type, msg->target.data_type, (int)(minimum*50));
    if(!manageORLStatus(status,"setORLMovement"))
    {
      return false;
    }
  }

  setAlgorithmMode(MOVING, __FUNCTION__,__LINE__);
  jog_state_ = true;
#if ENABLE_ALGOMNGR_PRINTFS
printf("[jogTrjntFn,%d] jog_state TRUE algorithm_mode_ %d\n",__LINE__, algorithm_mode_);
#endif
  return true;
}

bool AlgorithmManager::movePauseFn(edo_core_msgs::MovementCommand * msg)
{
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[movePauseFn,%d] B PAUSE Move algorithm_mode_ %d alarm %d\n",__LINE__,algorithm_mode_,alarm_state_);
#endif
  if(moveInProgress_)
  {
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[movePauseFn,%d] ORL_stop_motion algorithm_mode_ %d\n",__LINE__,algorithm_mode_);
#endif
    int status = ORL_stop_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    if(!manageORLStatus(status,"ORL_stop_motion"))
    {
      return false;
    }
#if ENABLE_ROS_WARN
    ROS_WARN("stop requested, enter in pause");
#endif
    pause_state_ = true;
    // The ack to the command will be sent when the MOVE stopped
  }
  else if(algorithm_mode_ == WAITING)
  {
    // Request to PAUSE while waiting for the delay expiration
    waiting_ = false;

    setAlgorithmMode(PAUSE, __FUNCTION__,__LINE__);
    feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 13, __FUNCTION__, __LINE__);	// This is for the MOVE waiting
    feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 40, __FUNCTION__, __LINE__);	// This is for the end of the PAUSE command 
  }
  else
  {
    setAlgorithmMode(PAUSE, __FUNCTION__,__LINE__);
    feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 40, __FUNCTION__, __LINE__);	// This is for the end of the PAUSE command
  }
  
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[movePauseFn,%d] E PAUSE Move algorithm_mode_ %d\n",__LINE__,algorithm_mode_);
#endif
  return true;
}

bool AlgorithmManager::moveResumeFn(edo_core_msgs::MovementCommand * msg)
{  
  int status;
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[%s,%d] B RESUME Move algorithm_mode_ %d Alarm %d Stopped %d\n",__FILE__,__LINE__,algorithm_mode_,alarm_state_,moveStopped_);
#endif
  /*
  * Do not accept incoming requests of resume motion is ORL has not set the Initial position 
  */
  if ((alarm_state_ == true) || (first_time_ == true)) {
    feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 0, __FUNCTION__, __LINE__);
  }
  else if (algorithm_mode_ == PAUSE)
  {
    if (moveStopped_)
    {
      feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 50, __FUNCTION__, __LINE__);  // Reply to the RESUME request
#if ENABLE_ROS_WARN
    ROS_WARN("enter moveResumeFn");
#endif
      setAlgorithmMode(RECOVERY, __FUNCTION__,__LINE__);
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[moveResumeFn,%d] ORL_resume_motion algorithm_mode_ %d\n",__LINE__,algorithm_mode_);
#endif
      status = ORL_resume_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);

      pause_state_ = false;
      setAlgorithmMode(MOVING, __FUNCTION__,__LINE__);
    }
    else if (!MoveCommandMsgListInOrl_.empty())
    {
      feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 50, __FUNCTION__, __LINE__);
#if ENABLE_ROS_WARN
    ROS_WARN("enter moveResumeFn");
#endif
      setAlgorithmMode(RECOVERY, __FUNCTION__,__LINE__);
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[moveResumeFn,%d] ORL_resume_motion algorithm_mode_ %d\n",__LINE__,algorithm_mode_);
#endif
      status = ORL_resume_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);

      pause_state_ = false;
      setAlgorithmMode(MOVING, __FUNCTION__,__LINE__);
    }      
    else
    {
      feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 50, __FUNCTION__, __LINE__);
      setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__);
		  pause_state_ = false;
    }
  }
  else
  {
    feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 50, __FUNCTION__, __LINE__);
  }  
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[moveResumeFn,%d] E RESUME Move algorithm_mode_ %d\n",__LINE__,algorithm_mode_);
#endif
  return true;
}

bool AlgorithmManager::moveCancelFn(edo_core_msgs::MovementCommand * msg)
{
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[moveCancelFn,%d] B CANCEL Move algorithm_mode_ %d\n",__LINE__,algorithm_mode_);
#endif
	if (algorithm_mode_ == PAUSE)
	{
		int status;

		setAlgorithmMode(RECOVERY, __FUNCTION__,__LINE__);
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[moveCancelFn,%d] ORL_cancel_motion algorithm_mode_ %d\n",__LINE__,algorithm_mode_);
#endif
		status = ORL_cancel_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__); //finished????
		pause_state_ = false;
		pending_cancel_ = false;
		waiting_ = false;
    feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 60, __FUNCTION__, __LINE__);
    while (!MoveCommandMsgListInOrl_.empty())
    {
      MoveCommandMsgListInOrl_.pop_front();
      // For each Move Command popped from ORL give a feedback to machine_state/bridge
      feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 20, __FUNCTION__, __LINE__); 
    }
	}
	else if (algorithm_mode_ == WAITING)
	{
		// Request to CANCEL while waiting for the delay expiration
		setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__);
		delay_ = 255;
		waiting_ = false;
    feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 60, __FUNCTION__, __LINE__);
	}
	else
	{
		// If I can't satisfy the request now, I remember it.
		pending_cancel_ = true;
	}
	
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[moveCancelFn,%d] E Cancel Move algorithm_mode_ %d\n",__LINE__,algorithm_mode_);
#endif
 	return true;
}

//funzione di jog in spazio cartesiano
bool AlgorithmManager::jogCarlinFn(edo_core_msgs::MovementCommand *  msg)
{
  int status = 0;
  unsigned int sj_lastJointAxis;
  unsigned long sm_joints_mask;
  unsigned long sm_joints_aux_mask = joints_aux_mask_;
  double delta = 0;
  double minimum = 1.0;
  ORL_cartesian_position temp_cart_pos, p_new, p_step;
  ORL_joint_value current_state_local;
  float R_new[3][3];

#if ENABLE_ROS_INFO
  ROS_INFO("[%d] Move_command:%c move_type:%c", __LINE__, msg->move_command, msg->move_type);
  ROS_INFO("[%d] Target.data_type:%c", __LINE__, msg->target.data_type);
#endif

  if ((msg->target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION) &&
      (msg->target.data_type != E_MOVE_DEST_POINT::E_MOVE_POINT_XTND_POS))
    return false;

  // Evaluate the current joint position
  current_state_local.unit_type = ORL_POSITION_LINK_DEGREE;
  for(size_t i = 0; i < ORL_MAX_AXIS; i++)
  {
     if(i == N_AX_GRIPPER)
      current_state_local.value[i]=std::max(strk_[IDX_STRK_ENP_N][i] + 0.2, std::min(strk_[IDX_STRK_ENP_P][i]-0.2, current_state_.value[i]));
    else
      current_state_local.value[i]=std::max(strk_[IDX_STRK_ENP_N][i] + 0.1, std::min(strk_[IDX_STRK_ENP_P][i]-0.1, current_state_.value[i]));
  }

  delta  = msg->target.cartesian_data.x * msg->target.cartesian_data.x;
  delta += msg->target.cartesian_data.y * msg->target.cartesian_data.y;
  delta += msg->target.cartesian_data.z * msg->target.cartesian_data.z;
  delta += msg->target.cartesian_data.a * msg->target.cartesian_data.a;
  delta += msg->target.cartesian_data.e * msg->target.cartesian_data.e;
  delta += msg->target.cartesian_data.r * msg->target.cartesian_data.r;

  if (msg->target.data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION)
  {
    if (sm_joints_aux_mask != 0)
    {
      memcpy(&target_joint_, &current_state_local, sizeof(ORL_joint_value));
      target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
      
      sm_joints_mask = joints_mask_ & (unsigned long)msg->target.joints_mask;
      sj_lastJointAxis = CommonService::getLastJointAxis(sm_joints_mask);
      for(unsigned int i = 0; i < sj_lastJointAxis; i++)
      {
        if (sm_joints_aux_mask & (1 << i))
        {
          target_joint_.value[i] = current_state_local.value[i];
#if ENABLE_ROS_INFO
          ROS_INFO("Set aux axis %d to %f", i + 1,  target_joint_.value[i]);
#endif
        }
        else
        {
          target_joint_.value[i] = 0.0;
#if ENABLE_ROS_INFO
          ROS_INFO("Set     axis %d to %f", i + 1,  target_joint_.value[i]);
#endif
        }
      } 
      msg->target.data_type = E_MOVE_DEST_POINT::E_MOVE_POINT_XTND_POS; // Fake change of the destination position from 'position' to 'xtnd_position'
    }
  }
  else if (msg->target.data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_XTND_POS)
  {
    memcpy(&target_joint_, &current_state_local, sizeof(ORL_joint_value));
    target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
    {
      sm_joints_mask = joints_mask_ & (unsigned long)msg->target.joints_mask;
      sj_lastJointAxis = CommonService::getLastJointAxis(sm_joints_mask);
      for(unsigned int i = 0; i < sj_lastJointAxis; i++)
      {
        if (sm_joints_mask & (1 << i))
        {
          if(fabs(msg->target.joints_data[i]) > 0.01)
          {
            minimum=std::min(fabs(msg->target.joints_data[i]),minimum);
            
            if(i == N_AX_GRIPPER)
              target_joint_.value[i] = (msg->target.joints_data[i] < 0) ? strk_[IDX_STRK_ENP_N][i] + 0.2 : strk_[IDX_STRK_ENP_P][i] - 0.2;
            else
              target_joint_.value[i] = (msg->target.joints_data[i] < 0) ? strk_[IDX_STRK_ENP_N][i] + 0.1 : strk_[IDX_STRK_ENP_P][i] - 0.1;
            
// printf("[%d] Joint %d has a delta %f\n",__LINE__,i,target_joint_.value[i]);
          }
#if ENABLE_ROS_INFO
          ROS_INFO("Set     axis %d to %f", i + 1,  target_joint_.value[i]);
#endif
        }
        else
        {
          target_joint_.value[i] = 0.0;
        }
      }
    }
  }

  delta=sqrt(delta);
  if (delta < 0.01)
    delta = minimum;

  {
;
    if(!setReferenceFn(msg))
    {
      return false;
    }
  }

  status = ORL_direct_kinematics(&temp_cart_pos, &current_state_local, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
  if(!manageORLStatus(status,"ORL_direct_kinematics"))
  {
    return false;
  }
  
  p_.x = (msg->target.cartesian_data.x<0)?-1:((msg->target.cartesian_data.x>0)?+1:0);
  p_.y = (msg->target.cartesian_data.y<0)?-1:((msg->target.cartesian_data.y>0)?+1:0);
  p_.z = (msg->target.cartesian_data.z<0)?-1:((msg->target.cartesian_data.z>0)?+1:0);
  p_.a = (msg->target.cartesian_data.a<0)?-1:((msg->target.cartesian_data.a>0)?+1:0);
  p_.e = (msg->target.cartesian_data.e<0)?-1:((msg->target.cartesian_data.e>0)?+1:0);
  p_.r = (msg->target.cartesian_data.r<0)?-1:((msg->target.cartesian_data.r>0)?+1:0);
  
  temp_cart_pos.a *= M_PI / 180.0;
  temp_cart_pos.e *= M_PI / 180.0;
  temp_cart_pos.r *= M_PI / 180.0;
  
  vZYZm(temp_cart_pos, R_old_);
  
  p_step.x = p_.x;
  p_step.y = p_.y;
  p_step.z = p_.z;
  p_step.a = MAN_ORN_STEP * p_.a * M_PI / 180.0;
  p_step.e = MAN_ORN_STEP * p_.e * M_PI / 180.0;
  p_step.r = MAN_ORN_STEP * p_.r * M_PI / 180.0;
  
  vXYZm(p_step, R_step_); // 180418 GC & MB 

  memset(R_new, 0, sizeof(float)*9);
  
  for(int si_i = 0; si_i < 3; si_i++)
  {
    for(int si_j = 0; si_j < 3; si_j++)
    {
      for(int si_k = 0; si_k < 3; si_k++)
      {
        R_new[si_i][si_j] += R_step_[si_i][si_k] * R_old_[si_k][si_j];
      }
    }
  }
  
  mvZYZ(R_new, &p_new);
  
  temp_cart_pos.x += MAN_LIN_STEP * p_.x;
  temp_cart_pos.y += MAN_LIN_STEP * p_.y;
  temp_cart_pos.z += MAN_LIN_STEP * p_.z;
  temp_cart_pos.a = p_new.a*180.0 / M_PI;
  temp_cart_pos.e = p_new.e*180.0 / M_PI;
  temp_cart_pos.r = p_new.r*180.0 / M_PI;
  
  for(int si_i = 0; si_i < 3; si_i++)
    for(int si_j = 0; si_j < 3; si_j++)
      R_old_[si_i][si_j] = R_new[si_i][si_j];
    
  if (algorithm_mode_ != MOVING)
  {
    status = ORL_set_position(NULL, &current_state_local, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);  
    if(!manageORLStatus(status,"ORL_set_position"))
    {
      return false;
    }
  }
  
  memcpy(&target_cart_, &temp_cart_pos, sizeof(ORL_cartesian_position));
  {
    std::vector<int> move_parameters;
    move_parameters.resize(3);
    
    move_parameters[0] = ORL_FLY;
    move_parameters[1] = ORL_ADVANCE;
    move_parameters[2] = ORL_FLY_NORMAL;

    msg->move_command = E_MOVE_COMMAND::E_MOVE_COMMAND_JOGMOVE;
    msg->move_type = E_MOVE_TYPE::E_MOVE_TYPE_LINEAR;
    jog_target_data_type_ = msg->target.data_type;  // Remember the destination data type. Useful in Cartesian 
    status = setORLMovement(move_parameters, msg->move_type, msg->target.data_type, std::max((int)(50*delta),1));
    if(!manageORLStatus(status,"setORLMovement"))
    {
      return false;
    }
  }
  setAlgorithmMode(MOVING, __FUNCTION__,__LINE__);
  jog_carlin_state_ = true;
  jog_state_ = true;
  feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 2, __FUNCTION__, __LINE__);
  return true;
}

//funzione di jog stop
bool AlgorithmManager::jogStopFn(edo_core_msgs::MovementCommand *  msg)
{
	if (algorithm_mode_ == MOVING)
	{
		int status = ORL_stop_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		if(!manageORLStatus(status,"ORL_stop_motion"))
		{
			return false;
		}
	}

	return true;
}


void AlgorithmManager::initialize(edo_core_msgs::JointStateArrayConstPtr msg)
{
	joints_control_.size = msg->joints.size();
	joints_control_.joints.resize(joints_control_.size);
	for(size_t j_id = 0; j_id < joints_control_.size; j_id++)
	{
		hold_position_.value[j_id] = current_state_.value[j_id];
		joints_control_.joints[j_id].position = current_state_.value[j_id];
		joints_control_.joints[j_id].velocity = 0;
		joints_control_.joints[j_id].current = 0;
	}
	setAlgorithmMode(INITIALIZED, __FUNCTION__,__LINE__);
#if ENABLE_ROS_INFO
	ROS_INFO("First state received Algorithm is now initialized");
#endif
}


int AlgorithmManager::setORLMovement(std::vector<int> move_parameters, int movement_type, int point_data_type, int ovr)
{

	ORL_System_Variable Sysvar;
	int si_status;	
	ros::Duration elapsed_time;
	double elapsed_time_in_msec;
	ros::Time start_preliminary_count_time;
  
#if ENABLE_ROS_INFO
	ROS_INFO("set ORL movement");
	if (move_parameters.size() != 3 && move_parameters.size() != 4) 
	{
		ROS_ERROR("Invalid parameters...");
		return MOVETYPE_UNDEF;
	}
#endif
	
	if(ovr>0)
	{
		sprintf(Sysvar.sysvar_name, "$ARM_DATA[%d].ARM_OVR", ORL_ARM1 + 1); //in JOG rallento la macchina al 50%
		Sysvar.ctype = ORL_INT; //  ORL_BOOL  ORL_REAL  ORL_STRING
		Sysvar.iv = ovr;

		ORL_set_data   (Sysvar,        	  /* [IN]      Structure for the system variable */
						LOCAL_VERBOSITY , /* [IN]      Verbose ON/OFF */
						ORL_CNTRL01       /* [IN]      Controller Index */);
	}
	start_preliminary_count_time = ros::Time::now();
  if ((movement_type == E_MOVE_TYPE::E_MOVE_TYPE_JOINT) && (point_data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_JOINT))
  { // MOVE JOINT TO <JOINT_POS>
#if ENABLE_ROS_INFO
    ROS_INFO("MOVE JOINT TO <JOINT_POS>");
#endif
    si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRJNT, NULL, &target_joint_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
  }
  else if ((movement_type == E_MOVE_TYPE::E_MOVE_TYPE_JOINT) && (point_data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION))
  { // MOVE JOINT TO <CART_POSITION>
#if ENABLE_ROS_INFO
    ROS_INFO("MOVE JOINT TO <CART_POSITION>");
#endif
    si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRJNT, &target_cart_, NULL, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
  }
  else if ((movement_type == E_MOVE_TYPE::E_MOVE_TYPE_JOINT) && (point_data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_XTND_POS))
  { // MOVE JOINT TO <XTND_POSITION>
#if ENABLE_ROS_INFO
    ROS_INFO("MOVE JOINT TO <XTND_POSITION>");
#endif
    si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRJNT, &target_cart_, &target_joint_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
  }
  else if ((movement_type == E_MOVE_TYPE::E_MOVE_TYPE_LINEAR) && (point_data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_JOINT))
  { // MOVE LINEAR TO <JOINT_POS>
#if ENABLE_ROS_INFO
    ROS_INFO("MOVE LINEAR TO <JOINT_POS>");
#endif
    si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARLIN, NULL, &target_joint_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
  }
  else if ((movement_type == E_MOVE_TYPE::E_MOVE_TYPE_LINEAR) && (point_data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION))
  { // MOVE LINEAR TO <CART_POSITION>
#if ENABLE_ROS_INFO
    ROS_INFO("MOVE LINEAR TO <CART_POSITION>");
#endif
    si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARLIN, &target_cart_, NULL, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
  }
  else if ((movement_type == E_MOVE_TYPE::E_MOVE_TYPE_LINEAR) && (point_data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_XTND_POS))
  { // MOVE LINEAR TO <XTND_POSITION>
#if ENABLE_ROS_INFO
    ROS_INFO("MOVE LINEAR TO <XTND_POSITION>");
#endif
    si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARLIN, &target_cart_, &target_joint_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
  }
  else if ((movement_type == E_MOVE_TYPE::E_MOVE_TYPE_CIRCULAR) && (point_data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_JOINT))
  { // MOVE CIRCULAR TO <JOINT_POS> VIA <JOINT_POS>
#if ENABLE_ROS_INFO
    ROS_INFO("MOVE CIRCULAR TO <JOINT_POS> VIA <JOINT_POS>");
#endif
	if(move_parameters[3] == ORL_CIR_TARGET)
	{
	  si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARCIR, NULL, &target_joint_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    }  
	else if(move_parameters[3] == ORL_CIR_VIA)
	{
	  si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARCIR, NULL, &target_joint_via_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	}
  }
  else if ((movement_type == E_MOVE_TYPE::E_MOVE_TYPE_CIRCULAR) && (point_data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION))
  { // MOVE CIRCULAR TO <CART_POSITION> VIA <CART_POSITION>
#if ENABLE_ROS_INFO
    ROS_INFO("MOVE CIRCULAR TO <CART_POSITION> VIA <CART_POSITION>");
#endif
	if(move_parameters[3] == ORL_CIR_TARGET)
	{
	  si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARCIR, &target_cart_, NULL, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    }  
	else if(move_parameters[3] == ORL_CIR_VIA)
	{
	  si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARCIR, &target_cart_via_, NULL, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	}
  }
  else if ((movement_type == E_MOVE_TYPE::E_MOVE_TYPE_CIRCULAR) && (point_data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_XTND_POS))
  { // MOVE CIRCULAR TO <XTND_POSITION> VIA <XTND_POSITION>
#if ENABLE_ROS_INFO
    ROS_INFO("MOVE CIRCULAR TO <XTND_POSITION> VIA <XTND_POSITION>");
#endif
	if(move_parameters[3] == ORL_CIR_TARGET)
	{
	  si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARCIR, &target_cart_, &target_joint_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    }  
	else if(move_parameters[3] == ORL_CIR_VIA)
	{
	  si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARCIR, &target_cart_via_, &target_joint_via_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	}
  }
  else
  {
    ROS_ERROR("MOVE UNDEFINED");
    si_status = MOVETYPE_UNDEF; //Movement type undefined
  }
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
  elapsed_time = ros::Time::now() - start_preliminary_count_time;
  elapsed_time_in_msec = ((double)elapsed_time.toNSec()) / 1000000.0f;
  if (si_status == 0)
     printf ("[setORLMovement,%d] algorithm_mode_ %d DT:%f ms\n", __LINE__, algorithm_mode_, elapsed_time_in_msec);
  else
     printf ("[setORLMovement,%d] Status %d algorithm_mode_ %d DT:%f ms\n", __LINE__, si_status, algorithm_mode_,elapsed_time_in_msec);
#endif
  return si_status; 
}

void AlgorithmManager::keepPosition()
{
	for(uint8_t i = 0; i < joints_control_.size; i++)
	{
		joints_control_.joints[i].position = hold_position_.value[i];
		joints_control_.joints[i].velocity = 0.0;
		joints_control_.joints[i].current = hold_current_.current[i];
		joints_control_.joints[i].ff_velocity = 0.0;
		joints_control_.joints[i].ff_current = 0.0;
	}
}

bool AlgorithmManager::manageORLStatus(int const& status, const char* service_name)
{
	if(status < RET_OK) //Error!
	{
#if ENABLE_ALGOMNGR_PRINTFS
printf(  "<%s> ORL error <%s>\n",service_name,ORL_decode_Error_Code(status));
#endif
#if ENABLE_ROS_WARN
		ROS_WARN("<%s> ORL error <%s>\n",service_name,ORL_decode_Error_Code(status));
#endif
		ROS_ERROR("<%s> ORL error %d",service_name,status);
		
		switch(_ORL_error)
		{
			case -1:
				ORL_stop_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
				feedbackFn(MESSAGE_FEEDBACK::ERROR, status, __FUNCTION__, __LINE__);
				_ORL_error = 1;
			break;
			default:
				setAlgorithmMode(BLOCKED, __FUNCTION__,__LINE__);
				memcpy(&hold_position_, &current_state_, sizeof(ORL_joint_value));
				feedbackFn(MESSAGE_FEEDBACK::ERROR, status, __FUNCTION__, __LINE__);
			break;
		}
		
		return false;
	}

	return true;
}

void AlgorithmManager::feedbackFn(int type, int data, const char *apc_func, int asi_line)
{
	edo_core_msgs::MovementFeedback feedback;

  static const char *sam_MovementFeedback[6] = {
	"BUFFER_FULL     (-3)",
	"COMMAND_REJECTED(-2)", /* comando scartato */
	"ERROR           (-1)", /* l’esecuzione della move è stata interrotta per un errore */
	"COMMAND_RECEIVED( 0)", /* acknowledgement, comando ricevuto */
	"F_NEED_DATA     ( 1)", /* si chiede il punto successivo */
	"COMMAND_EXECUTED( 2)"  /* movimento completato */
  };

	feedback.type = type;
	feedback.data = data;
	feedback_publisher_.publish(feedback);
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
printf("feedbackFn [%s,%d] <%s> Data %d\n",apc_func,asi_line,sam_MovementFeedback[type-MESSAGE_FEEDBACK::BUFFER_FULL],data);  
#endif
}

edo_core_msgs::JointControlArray const& AlgorithmManager::getCurrentControl()
{
	return joints_control_;
}

void AlgorithmManager::updateControl()
{
  edo_core_msgs::MovementCommand msg;

  // Test if there are some msg in the msglist
  while (!msglist_.empty())
  {
#if ENABLE_ROS_INFO
    ROS_INFO("pop move command");
#endif

    // Get the first message from the back of the list, without pop it out
    // In this way, the commands are received before the moves.
    msg = msglist_.back();

    if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_MOVE)
    {
      if (alarm_state_ == false) 
      {  
        // Procedo con lo scheduling delle move solo se:
        // 1) Non sto attendendo la fine di una move, cioe' non sono nello stato di WAITING
        // 2) Oppure se la MOVE in corso NON e' una MOVE su cui dovrò attendere,
        //    in altre parole, schedulo la prossima MOVE solo se e' in fly
        
        // Sono nella fase di terminazione della move corrente o in uno stato non consistente con l'esecuzione della move
        if ((algorithm_mode_ == WAITING) || (algorithm_mode_ == PAUSE) || (algorithm_mode_ == FINISHED) || (algorithm_mode_ == BLOCKED))
          break; 
        // Sono durante una richiesta di stop a causa di una richiesta di PAUSE o di CANCEL con una move in corso
        // Questo ferma anche una eventuale MOVE successiva ad una MOVE in fly, ma comunque mi sto fermando.
        if ((pause_state_ == true) || (pending_cancel_ == true))
          break; 
        if (!MoveCommandMsgListInOrl_.empty()) 
        {
          edo_core_msgs::MovementCommand tmpMsg;
          tmpMsg = MoveCommandMsgListInOrl_.back();  
          if (tmpMsg.delay != 255)
            break;  // La prima move in esecuzione NON e' in fly
        }       
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
printf ("[updateControl,%d] GET item %c ML:%d MCLS %d Ax1:%f\n",__LINE__,msg.move_command, msglist_.size(), MoveCommandMsgListInOrl_.size(), msg.target.joints_data[0]);
#endif        
        msglist_.pop_back(); // pop the message out of the list from the back.
        //
        // All the below functions, change the state to MOVING without testing from which state I come from.
        // So it is mandatory that the current state must be consistent with the future state of MOVING.
        // i.e. The current state can't be PAUSE for example.
        //        
        if (msg.move_type == E_MOVE_TYPE::E_MOVE_TYPE_JOINT)
        {
          if(!moveTrjntFn(&msg))
          {
            ROS_ERROR("Move Jnt not executed...");
            feedbackFn(MESSAGE_FEEDBACK::ERROR, State::FAILED_MOVE, __FUNCTION__, __LINE__);
          }
        }
        else if (msg.move_type == E_MOVE_TYPE::E_MOVE_TYPE_LINEAR)
        {
          if(!moveCarlinFn(&msg))
          {
            ROS_ERROR("Move Lin not executed...");
            feedbackFn(MESSAGE_FEEDBACK::ERROR, State::FAILED_MOVE, __FUNCTION__, __LINE__);
          }
        }
        else if (msg.move_type == E_MOVE_TYPE::E_MOVE_TYPE_CIRCULAR)
        {
          if(!moveCarcirFn(&msg))
          {
            ROS_ERROR("Move Cir not executed...");
            feedbackFn(MESSAGE_FEEDBACK::ERROR, State::FAILED_MOVE, __FUNCTION__, __LINE__);
          }
        }
      }
      else
      {
        msglist_.pop_back();
        feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 1, __FUNCTION__, __LINE__);
      }
    }
    else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE)
    {
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
printf ("[updateControl,%d] GET item %c\n",__LINE__,msg.move_command);
#endif
      msglist_.pop_back();
      if (alarm_state_ == false) 
      {  
        if(!movePauseFn(&msg))
        {
          ROS_ERROR("Pause not executed...");
          feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR, __FUNCTION__, __LINE__);
        }
      }
      else
      {
        feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 0, __FUNCTION__, __LINE__);
      }
    }
    else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_RESUME)
    {
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
printf ("[updateControl,%d] GET item %c\n",__LINE__,msg.move_command);
#endif
      msglist_.pop_back();
      if (alarm_state_ == false) 
      { 
        if(!moveResumeFn(&msg))
        {
          ROS_ERROR("Resume not executed...");
          feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR, __FUNCTION__, __LINE__);
        }
      }
      else
      {
        feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 75, __FUNCTION__, __LINE__);
      } 
    }
    else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL)
    {
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
printf ("[updateControl,%d] GET item %c Lin %f\n",__LINE__,msg.move_command,msg.cartesian_linear_speed);
#endif
      msglist_.pop_back();
      if(!moveCancelFn(&msg))
      {
        ROS_ERROR("Cancel not executed...");
        feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR, __FUNCTION__, __LINE__);
      }
      while (!msglist_.empty())
      {
        msglist_.pop_back();
        // For each Move Command popped from ORL give a feedback to machine_state/bridge
        feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 20, __FUNCTION__, __LINE__); 
      }
    }
    else
    {
      ROS_WARN("Command not implemented, please use only accepted command type");
      feedbackFn(MESSAGE_FEEDBACK::ERROR, State::MOVETYPE_UNDEF, __FUNCTION__, __LINE__);
      return;
    }
  }

  //scodo i messaggi arrivati per il jog
  while (!msglistJog_.empty())
  {
    msg = msglistJog_.back();

    if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_JOGMOVE)
    {
      msglistJog_.pop_back();
      if (msg.move_type == E_MOVE_TYPE::E_MOVE_TYPE_JOINT)
      {
        if(!jogTrjntFn(&msg))
        {
          ROS_ERROR("Jog JNTtrj not executed...");
          feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR, __FUNCTION__, __LINE__);
        }
      }
      else if (msg.move_type == E_MOVE_TYPE::E_MOVE_TYPE_LINEAR)
      {
        if(!jogCarlinFn(&msg))
        {
          ROS_ERROR("Jog CARTtrj not executed...");
          feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR, __FUNCTION__, __LINE__);
        }
      }

      if (alarm_state_ == true)
      {
        if ((jog_state_ == true) && (alarm_exclusion_ == false))
        {
          start_jog_time_ = ros::Time::now();
          alarm_exclusion_ = true;
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
printf ("[updateControl,%d] Alarm exclusion started\n",__LINE__);
#endif
        }
      }
      else
      {
        alarm_exclusion_ = false;
      }
    }
    else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_JOGSTOP)
    {
      msglistJog_.pop_back();
      if(!jogStopFn(&msg))
      {
        ROS_ERROR("Move JOG stop not executed...");
        feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR, __FUNCTION__, __LINE__);
      }
    }
    else
    {
      msglistJog_.pop_back();
      ROS_WARN("Command not implemented, please use only accepted command type");
      feedbackFn(MESSAGE_FEEDBACK::ERROR, State::MOVETYPE_UNDEF, __FUNCTION__, __LINE__);
      return;
    }
  }

#if 0
######################################################################################################
  if ((alarm_state_ == true) && (jog_state_ == true) && (alarm_exclusion_ == true))
  {
    ros::Duration d = ros::Time::now() - start_jog_time_;
    if(d.toSec() >= (double)3.0)
    {
      alarm_exclusion_ = false;
      if (moveInProgress_)
      {
        (void)ORL_stop_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
      }
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
printf ("[updateControl,%d] Alarm exclusion ended moveInProgress_ %d\n",__LINE__,moveInProgress_);
#endif
    }
  }
######################################################################################################
#endif

  {
    // Publish algorithm_mode_
    std_msgs::Int8 temp_algo_state;
    temp_algo_state.data = algorithm_mode_;
    algorithm_state_pub_.publish(temp_algo_state);
  }
  
  if(algorithm_mode_ == MOVING)
  {
    setControl(true);
  }
  else if(algorithm_mode_ == INITIALIZED)
  {
    setControl(false);
    keepPosition();
  }
  else if (algorithm_mode_ == WAITING)
  {
    setControl(false);
    keepPosition();
    ros::Duration d = ros::Time::now() - start_wait_time_;
    if ((d.toSec() >= (double)delay_) || (alarm_state_ == true))
    {
      waiting_ = false;
      setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__);
      delay_ = 255;
      feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 13, __FUNCTION__, __LINE__);
      if (pause_state_)
      {
        setAlgorithmMode(PAUSE, __FUNCTION__,__LINE__);
      }
      else
      {
        setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__);
      }
    }
  }
  else if (algorithm_mode_ == BLOCKED)
  {
    setControl(false);
    keepPosition();
    waiting_ = false;
    pause_state_ = false;
    pending_cancel_ = false;
    setAlgorithmMode(INITIALIZED, __FUNCTION__,__LINE__);
  }
  else if (algorithm_mode_ == PAUSE)
  {
    setControl(false);
    keepPosition();
  }
  else if (algorithm_mode_ == FINISHED)
  {
    int status;

    setControl(false);
    if (jog_state_ || pending_cancel_)
    {
      setAlgorithmMode(RECOVERY, __FUNCTION__,__LINE__);
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[%s,%d] ORL_cancel_motion\n",__FUNCTION__,__LINE__);
#endif
      status = ORL_cancel_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
      (void)manageORLStatus(status,"ORL_cancel_motion"); // don't care the return status
      jog_state_ = false;
      pending_cancel_ = false;
      while (!MoveCommandMsgListInOrl_.empty())
      {
        MoveCommandMsgListInOrl_.pop_front();
        // For each Move Command popped from ORL give a feedback to machine_state/bridge
        feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 1, __FUNCTION__, __LINE__); 
      }
    }

    keepPosition();
    setAlgorithmMode(INITIALIZED, __FUNCTION__,__LINE__);
  }
  return;
}

void AlgorithmManager::setControl(bool bactive)
{
	ORL_joint_value burned;
	burned.unit_type = ORL_POSITION_LINK_DEGREE;
  // edo_core_msgs::MovementCommandConstPtr msg;
  edo_core_msgs::MovementCommand msg;
	int status = 0;
	int status_dyn_mod = 0;
  int moveStarted,
      moveStopped,
      moveEnded,
      moveDecPhase,
      moveFly,
      moveFlyStarted,
      moveHandle,
      moveFlyHandle;

  if (bactive == false)
  {

    ORL_getMoveStatus (&moveStarted,    /* [OUT]  If True the actual movement is started */
                       &moveStopped,    /* [OUT]  If True the actual movement is stopped */
                       &moveEnded,      /* [OUT]  If True the actual movement is ended */
                       &moveDecPhase,   /* [OUT]  If True the actual movement is in deceleration phase */
                       &moveFly,        /* [OUT]  If True a second movement is already scheduled and ready to start */
                       &moveFlyStarted, /* [OUT]  If True, if a second movement is already scheduled, indicate that the fly connection is started */
                       &moveHandle,     /* [OUT]  Move Handle */
                       &moveFlyHandle,  /* [OUT]  Fly Move Handle */
                       ORL_CNTRL01, ORL_ARM1);
    if (idleMoveHandle_ == 0)
      idleMoveHandle_ = moveHandle;  // Initialize the idle move handle
    if (idleMoveHandle_ == moveHandle)
      moveInProgress_ = false;
    else
      moveInProgress_ = true;
    return;
  }
  
	for(size_t i = 0; i < NR_GET_NEXT_STEP_CALLS && status != FINAL_STEP; i++)
	{
		numberSteps_++;

		status = ORL_get_next_interpolation_step(&burned, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);

    if ((status >= 0) || (status == ORL_STATUS_IDLE))
    {
      ORL_getMoveStatus (&moveStarted,    /* [OUT]  If True the actual movement is started */
                         &moveStopped,    /* [OUT]  If True the actual movement is stopped */
                         &moveEnded,      /* [OUT]  If True the actual movement is ended */
                         &moveDecPhase,   /* [OUT]  If True the actual movement is in deceleration phase */
                         &moveFly,        /* [OUT]  If True a second movement is already scheduled and ready to start */
                         &moveFlyStarted, /* [OUT]  If True, if a second movement is already scheduled, indicate that the fly connection is started */
                         &moveHandle,     /* [OUT]  Move Handle */
                         &moveFlyHandle,  /* [OUT]  Fly Move Handle */
                         ORL_CNTRL01, ORL_ARM1);
      if (idleMoveHandle_ == moveHandle)
        moveInProgress_ = false;
      else
        moveInProgress_ = true;
      if (moveStarted)
        moveStopped_ = false;

      if (( moveHandle == previousMoveFlyHandle_ ) && (jog_state_ == false))
      {
        msg = MoveCommandMsgListInOrl_.front();
        // Move Command List Size MCLS
        MoveCommandMsgListInOrl_.pop_front();  // Now ORL has no more this Move Command
#if ENABLE_ALGOMNGR_PRINTFS
        printf("[setControl,%d] Step:%d EndMoveFLY ORL MCLS %d Ax1:%f\n",__LINE__, numberSteps_, MoveCommandMsgListInOrl_.size(), msg.target.joints_data[0]);
#endif
        feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 10, __FUNCTION__, __LINE__);
      }
      previousMoveFlyHandle_ = moveFlyHandle;
    }

    int status_dyn;
    ORL_Dynamic_Model sx_dyn_mod_fake;
	memset(&sx_dyn_mod_fake, 0, sizeof(ORL_Dynamic_Model));
    
    if(numberSteps_ > FILTER_STEP)
	{
      status_dyn_mod = ORL_get_dynamic_model(NULL, NULL, NULL, &sx_dyn_mod_, 1, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	  //printf("[DYM_real_SETCONTROL] current %g,%g,%g,%g,%g,%g\n",sx_dyn_mod_.current [ORL_AX1],sx_dyn_mod_.current [ORL_AX2],sx_dyn_mod_.current [ORL_AX3],sx_dyn_mod_fake.current [ORL_AX4],sx_dyn_mod_.current [ORL_AX5],sx_dyn_mod_.current [ORL_AX6]);
    }
    else
	{
      status_dyn = ORL_get_dynamic_model(NULL, NULL, NULL, &sx_dyn_mod_fake, 1, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	  //printf("[DYM_fake_SETCONTROL] current %g,%g,%g,%g,%g,%g\n",sx_dyn_mod_fake.current [ORL_AX1],sx_dyn_mod_fake.current [ORL_AX2],sx_dyn_mod_fake.current [ORL_AX3],sx_dyn_mod_fake.current [ORL_AX4],sx_dyn_mod_fake.current [ORL_AX5],sx_dyn_mod_fake.current [ORL_AX6]);
    }
    
#if ENABLE_DYNAMIC_MODEL_PRINTFS
		if(status_dyn_mod != ORLOPEN_RES_OK)
		{
			printf("[setControl, %d] %d",__LINE__, status_dyn_mod);			
		}
		else
		{
			printf("[DYM] current %g,%g,%g,%g,%g,%gnn",sx_dyn_mod_.current [ORL_AX1],sx_dyn_mod_.current [ORL_AX2],sx_dyn_mod_.current [ORL_AX3],sx_dyn_mod_.current [ORL_AX4],sx_dyn_mod_.current [ORL_AX5],sx_dyn_mod_.current [ORL_AX6]);
            printf("[DYM] tau_frict %g,%g,%g,%g,%g,%gnn",sx_dyn_mod_.tau_frict [ORL_AX1],sx_dyn_mod_.tau_frict [ORL_AX2],sx_dyn_mod_.tau_frict [ORL_AX3],sx_dyn_mod_.tau_frict [ORL_AX4],sx_dyn_mod_.tau_frict [ORL_AX5],sx_dyn_mod_.tau_frict [ORL_AX6]);
			printf("[DYM] tau_link %g,%g,%g,%g,%g,%gnn",sx_dyn_mod_.tau_link [ORL_AX1],sx_dyn_mod_.tau_link [ORL_AX2],sx_dyn_mod_.tau_link [ORL_AX3],sx_dyn_mod_.tau_link [ORL_AX4],sx_dyn_mod_.tau_link [ORL_AX5],sx_dyn_mod_.tau_link [ORL_AX6]);
			printf("[DYM] tau_motor %g,%g,%g,%g,%g,%gnn",sx_dyn_mod_.tau_motor [ORL_AX1],sx_dyn_mod_.tau_motor [ORL_AX2],sx_dyn_mod_.tau_motor [ORL_AX3],sx_dyn_mod_.tau_motor [ORL_AX4],sx_dyn_mod_.tau_motor [ORL_AX5],sx_dyn_mod_.tau_motor [ORL_AX6]);
		}
#endif	

		if ((status == 1) || (status == 2) || ((status < 0) && (status != ORL_STATUS_IDLE)))
		{
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
				printf("[ORL_GNIS, %d] Step:%d Steps_M42:%d status:%d\n",__LINE__, numberSteps_, numberSteps_M42_, status);
#endif
        if (status != 1) {
          numberSteps_ = numberSteps_M42_ = 0;
        }
		}
    
		if (status == ORL_STATUS_IDLE)
		{
				++numberSteps_M42_; // The interpolator has not an active move. This is not really an error, so I do not take care of it.
		}
		else if (status < 0)
		{
				if(!manageORLStatus(status,"ORL_get_next_interpolation_step"))
				{
						ROS_ERROR("ORL severe error during interpolation");
						break;
				}
		}
		else if (status == FINAL_STEP)
		{
			/* Handle the alarm state: set the position and update the dynamic model just on the final step */
			if (alarm_state_ == true)
			{			
				dynamicModelReset();
			}
			
			if (moveEnded == true)
			{
				if (alarm_state_ == false)
				{
					if (jog_state_) 
					{
						setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__);
						feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 1, __FUNCTION__, __LINE__);
					}
					else
					{
						if (!MoveCommandMsgListInOrl_.empty()) 
						{
							msg = MoveCommandMsgListInOrl_.front();
							MoveCommandMsgListInOrl_.pop_front();  // Now ORL has no more this Move Command
							if (pause_state_ == true)
							{
								feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 11, __FUNCTION__, __LINE__);
								feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 1, __FUNCTION__, __LINE__);
								pause_state_ == false;
								setAlgorithmMode(PAUSE, __FUNCTION__,__LINE__);
							}
							else if ((msg.delay != 255) && (msg.delay != 0))
							{
								delay_ = msg.delay;
								start_wait_time_ = ros::Time::now();
								setAlgorithmMode(WAITING, __FUNCTION__,__LINE__);
							}
							else
							{
								// Se ci sono MOVEs in ORL continue
								if (MoveCommandMsgListInOrl_.empty()) 
								{
									feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 11, __FUNCTION__, __LINE__);
									setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__);
								}
								else
								{
									while(!MoveCommandMsgListInOrl_.empty())
									{
										// Move Command List Size MCLS
										MoveCommandMsgListInOrl_.pop_front();  // Now ORL has no more this Move Command
										feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 10, __FUNCTION__, __LINE__);
									}
									feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 11, __FUNCTION__, __LINE__);
									setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__);
								}
							}
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
							printf("[setControl,%d] Step:%d EndMove ORL ML:%d MCLS %d Ax1:%f\n",__LINE__, numberSteps_, msglist_.size(), MoveCommandMsgListInOrl_.size(), msg.target.joints_data[0]);
#endif		
						}
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
						else
						{
							printf("[setControl,%d] Step:%d No Moves ORL MCLS %d\n",__LINE__, MoveCommandMsgListInOrl_.size());
						}
#endif	
					}
				}
				moveStopped_ = false;
				jog_carlin_state_ = false;
			}
			
			if (moveStopped == true)
			{   
				if (alarm_state_ == false)
				{
					if (jog_state_) 
					{
						feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 1, __FUNCTION__, __LINE__);
						setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__);
					}
					else
					{
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
						printf("[setControl,%d] Move stopped ORL MCLS %d\n",__LINE__,MoveCommandMsgListInOrl_.size());
#endif	
						if (pause_state_ == true)
						{
							feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 1, __FUNCTION__, __LINE__);
							pause_state_ == false;
							setAlgorithmMode(PAUSE, __FUNCTION__,__LINE__);
						}
						else if (pending_cancel_ == true)
						{
							feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 1, __FUNCTION__, __LINE__);
							(void)ORL_cancel_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
							(void)manageORLStatus(status,"ORL_cancel_motion"); // don't care the return status
							pending_cancel_ = false;
							waiting_ = false;
							while (!MoveCommandMsgListInOrl_.empty())
							{
								MoveCommandMsgListInOrl_.pop_front();
								// For each Move Command popped from ORL give a feedback to machine_state/bridge
								feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 20, __FUNCTION__, __LINE__); 
							}        
								setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__);
						}
						else 
						{
							setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__);
						}
					}	
				}
				jog_carlin_state_ = false;
				moveStopped_ = true;
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
				printf("[setControl,%d] Step:%d StopMove ORL MCLS %d\n",__LINE__, numberSteps_, MoveCommandMsgListInOrl_.size());
#endif
			}	
		}
		else if (status == NEED_DATA)
		{
			if (alarm_state_ == false)
			{
				if (jog_carlin_state_)
				{
					if (!updateJogCarlin())
					{
						ROS_ERROR("Error in JOG Cartesian");
						_ORL_error = -1;
					}
				}
				else
				{
					if (pause_state_ == false) 
					{
						if (MoveCommandMsgListInOrl_.size() < 3)
						{
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
			printf("[setControl,%d] Step:%d NeedNextMoveFLY ORL MCLS %d\n",__LINE__, numberSteps_, MoveCommandMsgListInOrl_.size());
#endif	
						feedbackFn(MESSAGE_FEEDBACK::F_NEED_DATA, 12, __FUNCTION__, __LINE__);
						}
					}
				}
			}
		}
		interpolation_data_.push_back(burned);
		}

  if (alarm_state_ == false)
  {
    if (interpolation_data_.size() < 3) 
    {
      ROS_ERROR("Buffer circolare parzialmente vuoto!!!");
    }
    else
    {
      for(size_t i = 0; i < joints_control_.joints.size(); i++)
      {
        if (status == FINAL_STEP)
        {
          joints_control_.joints[i].position = interpolation_data_.back().value[i];
        }
        else
        {
          joints_control_.joints[i].position = interpolation_data_[1].value[i];
        }

        float d_angle1 = interpolation_data_[1].value[i] - interpolation_data_[0].value[i];
        float d_angle2 = interpolation_data_[2].value[i] - interpolation_data_[1].value[i];

        hold_position_.value[i] = joints_control_.joints[i].position;
        hold_current_.current[i] = sx_dyn_mod_.current[i];

        joints_control_.joints[i].velocity    = (d_angle1 + d_angle2) / DUE_PER_TS;
        joints_control_.joints[i].current     = sx_dyn_mod_.current[i];
        joints_control_.joints[i].ff_velocity = joints_control_.joints[i].velocity;
        // joints_control_.joints[i].ff_current  = 0.0f;
      }
    }
  }
  else
  {
    // Do not touch the position, which is the last target position just before entering in the alarm state
    // In alarm state force velocity etc to zero.
    for(size_t i = 0; i < joints_control_.joints.size(); i++)
    {
      joints_control_.joints[i].velocity    = 0.0f;
      // joints_control_.joints[i].current     = hold_current_.current[i];
      joints_control_.joints[i].ff_velocity = 0.0f;
      // joints_control_.joints[i].ff_current  = 0.0f;
    }
  }
}

int AlgorithmManager::get_Strk(float strk[2][ORL_MAX_AXIS])
{
	ORL_System_Variable     orl_sys_var;

	memset(&orl_sys_var, 0x00, sizeof(ORL_System_Variable));

	for(int si_ax = 0; si_ax < ORL_MAX_AXIS; si_ax++)
	{
		sprintf(orl_sys_var.sysvar_name, "$ARM_DATA[%d].STRK_END_N[%d]", ORL_ARM1 + 1, si_ax + 1);
		orl_sys_var.ctype = ORL_INT; //  ORL_BOOL  ORL_REAL  ORL_STRING
		if(ORL_get_data(&orl_sys_var, LOCAL_VERBOSITY, ORL_CNTRL01) == ORLOPEN_RES_OK)
		{
			// for this version the 5th limit is set to 103.5 but here is loaded as integer, so its value is adjusted
			if(si_ax == 4)
			{
			  strk[0][si_ax] = orl_sys_var.iv-0.5;
			  #if ENABLE_ALGOMNGR_PRINTFS
			  printf("%f\n",strk[0][si_ax]);
			  #endif
			}
			else
			{
			  strk[0][si_ax] = orl_sys_var.iv;
			  #if ENABLE_ALGOMNGR_PRINTFS
			  printf("%f\n",strk[0][si_ax]);
			  #endif
			}
		}
	}
	for(int si_ax = 0; si_ax < ORL_MAX_AXIS; si_ax++)
	{
		sprintf(orl_sys_var.sysvar_name, "$ARM_DATA[%d].STRK_END_P[%d]", ORL_ARM1 + 1, si_ax + 1);
		orl_sys_var.ctype = ORL_INT; //  ORL_BOOL  ORL_REAL  ORL_STRING
		if(ORL_get_data(&orl_sys_var, LOCAL_VERBOSITY, ORL_CNTRL01) == ORLOPEN_RES_OK)
		{
			// for this version the 5th limit is set to 103.5 but here is loaded as integer, so its value is adjusted
			if(si_ax == 4)
			{
			  strk[1][si_ax] = orl_sys_var.iv+0.5;
			  #if ENABLE_ALGOMNGR_PRINTFS
			  printf("%f\n",strk[0][si_ax]);
			  #endif
			}
			else
			{
			  strk[1][si_ax] = orl_sys_var.iv;
			  #if ENABLE_ALGOMNGR_PRINTFS
			  printf("%f\n",strk[0][si_ax]);
			  #endif
			}
		}
	}
	return RET_OK;
}

void AlgorithmManager::vZYZm(ORL_cartesian_position p, float R[3][3])
{
	double ag_sn[3];
	double ag_cn[3];
	double g_ang;

	if(p.a == 0 && p.e == 0 && p.r == 0)
	{
		memset(R, 0x00, sizeof(float) * 9);

		R[0][0] = 1.0;
		R[1][1] = 1.0;
		R[2][2] = 1.0;
	}
	else
	{

		g_ang = (double)p.a;
		ag_sn[0] = sin(g_ang);
		ag_cn[0] = cos(g_ang);
		g_ang = (double)p.e;
		ag_sn[1] = sin(g_ang);
		ag_cn[1] = cos(g_ang);
		g_ang = (double)p.r;
		ag_sn[2] = sin(g_ang);
		ag_cn[2] = cos(g_ang);

		R[0][0] = (float)(ag_cn[0] * ag_cn[1] * ag_cn[2] - ag_sn[0] * ag_sn[2]);
		R[0][1] = (float)(-(ag_sn[0] * ag_cn[2] + ag_cn[0] * ag_cn[1] * ag_sn[2]));
		R[0][2] = (float)(ag_cn[0] * ag_sn[1]);

		R[1][0] = (float)(ag_sn[0] * ag_cn[1] * ag_cn[2] + ag_cn[0] * ag_sn[2]);
		R[1][1] = (float)(ag_cn[0] * ag_cn[2] - ag_sn[0] * ag_cn[1] * ag_sn[2]);
		R[1][2] = (float)(ag_sn[0] * ag_sn[1]);

		R[2][0] = (float)(-(ag_sn[1] * ag_cn[2]));
		R[2][1] = (float)(ag_sn[1] * ag_sn[2]);
		R[2][2] = (float)(ag_cn[1]);
	}

}

void AlgorithmManager::mvZYZ(float R[3][3], ORL_cartesian_position *p)
{
	float sf_next_1p0;
	float sf_next_0p1;

	sf_next_1p0 = ((float)(0.999995));
	sf_next_0p1 = ((float)(0.0017));     /* 0.1 deg */

										 /* OI936: 0.99999995 era mksw_fcst.sf_eulnear1p0 */
										 /* NEXP199: 0.99999991 era 0.99999995 */
	if(R[2][2] > (float)0.99999991)
		R[2][2] = 1.0;
	else if(R[2][2] < (float)-0.99999991)
		R[2][2] = -1.0;

	p->e = (float)acos((double)R[2][2]);
	if(fabs(R[2][2]) >(float)0.99999991)
	{
		p->a = 0.0;
		p->r = (float)atan2(R[1][0], R[1][1]);
	}
	else
	{
		p->a = atan2(R[1][2], R[0][2]);

		p->r = (float)atan2(R[2][1], -R[2][0]);

	}
}

void AlgorithmManager::vXYZm(ORL_cartesian_position p, float R[3][3])
{
	double ag_sn[3];
	double ag_cn[3];
	double g_ang;


	if(p.a == 0 && p.e == 0 && p.r == 0)
	{
		memset(R, 0x00, sizeof(float) * 9);

		R[0][0] = 1.0;
		R[1][1] = 1.0;
		R[2][2] = 1.0;
	}
	else
	{
		g_ang = (double)p.a;     /* Rotazione intorno a X */
		ag_sn[0] = sin(g_ang);
		ag_cn[0] = cos(g_ang);
		g_ang = (double)p.e;     /* Rotazione intorno a Y */
		ag_sn[1] = sin(g_ang);
		ag_cn[1] = cos(g_ang);
		g_ang = (double)p.r;     /* Rotazione intorno a Z */
		ag_sn[2] = sin(g_ang);
		ag_cn[2] = cos(g_ang);

		R[0][0] = (float)(ag_cn[1] * ag_cn[2]);
		R[0][1] = (float)(-ag_cn[0] * ag_sn[2] + ag_sn[0] * ag_sn[1] * ag_cn[2]);
		R[0][2] = (float)(ag_sn[0] * ag_sn[2] + ag_cn[0] * ag_sn[1] * ag_cn[2]);

		R[1][0] = (float)(ag_cn[1] * ag_sn[2]);
		R[1][1] = (float)(ag_cn[0] * ag_cn[2] - ag_sn[0] * ag_sn[1] * ag_sn[2]);
		R[1][2] = (float)(-ag_sn[0] * ag_cn[2] + ag_cn[0] * ag_sn[1] * ag_sn[2]);

		R[2][0] = (float)(-ag_sn[1]);
		R[2][1] = (float)(ag_sn[0] * ag_cn[1]);
		R[2][2] = (float)(ag_cn[0] * ag_cn[1]);
	}
}

edo_core_msgs::CartesianPose AlgorithmManager::computeCartesianPose(ORL_joint_value *joint_state)
{
	edo_core_msgs::CartesianPose cartesian_pose; //IL: why is not a heap variable?
	ORL_cartesian_position cartesian_state;
    
	memset(&cartesian_state, 0, sizeof(ORL_cartesian_position));
		
	if (noCartPose_ == false)
	{
		cartesian_state.unit_type = ORL_CART_POSITION;

		pthread_mutex_lock(&control_mutex_);

		int status = ORL_direct_kinematics(&cartesian_state, joint_state ,LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);//LOCAL_VERBOSITY
		if (status < RET_OK) 
		{
			#if ENABLE_ALGOMNGR_PRINTFS
			ROS_ERROR("Error in direct kinematics"); 
			#endif
		}

		pthread_mutex_unlock(&control_mutex_);
	}
		
	cartesian_pose.x = cartesian_state.x;
	cartesian_pose.y = cartesian_state.y;
	cartesian_pose.z = cartesian_state.z;
	cartesian_pose.a = cartesian_state.a;
	cartesian_pose.e = cartesian_state.e;
	cartesian_pose.r = cartesian_state.r;
	cartesian_pose.config_flags = cartesian_state.config_flags;	
	strncpy(target_cart_.config_flags, cartesian_state.config_flags, SSM_CONFIG_FLAG_STRLEN_MAX);
	
	return cartesian_pose;
}

edo_core_msgs::JointsPositions AlgorithmManager::computeJointValue(edo_core_msgs::CartesianPose cartesian_pose)
{
	ORL_joint_value joint_state;
	ORL_cartesian_position cartesian_state;
	
	memset(&joint_state, 0, sizeof(ORL_joint_value));
	joint_state.unit_type = ORL_POSITION_LINK_DEGREE;

	cartesian_state.unit_type = ORL_CART_POSITION;
	cartesian_state.x = cartesian_pose.x;
	cartesian_state.y = cartesian_pose.y;
	cartesian_state.z = cartesian_pose.z;
	cartesian_state.a = cartesian_pose.a;
	cartesian_state.e = cartesian_pose.e;
	cartesian_state.r = cartesian_pose.r;
	memcpy(cartesian_state.config_flags, cartesian_pose.config_flags.c_str(), sizeof(cartesian_state.config_flags));
	
	pthread_mutex_lock(&control_mutex_);
  
	int status = ORL_inverse_kinematics(&cartesian_state, &joint_state, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);//LOCAL_VERBOSITY
	if (status < RET_OK) 
	{
		#if ENABLE_ALGOMNGR_PRINTFS
		ROS_ERROR("Error in inverse kinematics"); 
		#endif
	}
  
	pthread_mutex_unlock(&control_mutex_);

	edo_core_msgs::JointsPositions joints_position;
	joints_position.positions.resize(joints_number_);
	for (size_t i = 0; i < joints_number_; i++) 
	{
		joints_position.positions[i] = joint_state.value[i];
	}
	return joints_position;
}

bool AlgorithmManager::updateJogCarlin()
{

	float R_new[3][3];
	ORL_cartesian_position p_new, temp_cart_pos;
	memset(R_new, 0x00, sizeof(float) * 9);
	for(int si_i = 0; si_i<3; si_i++)
		for(int si_j = 0; si_j < 3; si_j++)
		{
			for(int si_k = 0; si_k < 3; si_k++)
			{
				R_new[si_i][si_j] += R_step_[si_i][si_k] * R_old_[si_k][si_j];
			}
		}
	mvZYZ(R_new, &p_new);
	memcpy(&temp_cart_pos, &target_cart_, sizeof(ORL_cartesian_position));
	temp_cart_pos.x += MAN_LIN_STEP * p_.x;
	temp_cart_pos.y += MAN_LIN_STEP * p_.y;
	temp_cart_pos.z += MAN_LIN_STEP * p_.z;
	temp_cart_pos.a = p_new.a*180.0 / M_PI;
	temp_cart_pos.e = p_new.e*180.0 / M_PI;
	temp_cart_pos.r = p_new.r*180.0 / M_PI;

	std::vector<int> move_parameters;
	move_parameters.resize(3);

	move_parameters[0] = ORL_FLY;
	move_parameters[1] = ORL_ADVANCE;
	move_parameters[2] = ORL_FLY_NORMAL;
	memcpy(&target_cart_, &temp_cart_pos, sizeof(ORL_cartesian_position));
	int temp_status = setORLMovement(move_parameters, E_MOVE_TYPE::E_MOVE_TYPE_LINEAR, jog_target_data_type_, 0);

	if(!manageORLStatus(temp_status,"setORLMovement"))
	{
		return false;
	}
	
	for(int si_i = 0; si_i < 3; si_i++)
		for(int si_j = 0; si_j < 3; si_j++)
			R_old_[si_i][si_j] = R_new[si_i][si_j];
			
	return true;
}

bool AlgorithmManager::SwitchControl(edo_core_msgs::ControlSwitch::Request &req, edo_core_msgs::ControlSwitch::Response &res)
{
	if(req.mode == 1 && algorithm_mode_ != SWITCHED_OFF) {// Client requests to take control of the robot
		res.result = 0;
		setAlgorithmMode(SWITCHED_OFF, __FUNCTION__,__LINE__);  // Stop control
	} else if(req.mode == 2 && algorithm_mode_ == SWITCHED_OFF) {// Client requests to release the control
		res.result = 0;
		setAlgorithmMode(UNINITIALIZED, __FUNCTION__,__LINE__); // Ready to initialize ORL
	} else {
		res.result = 1;
	}
	return true;
}

bool AlgorithmManager::collisionCheck(float vr_CurMis, float vr_CurDyn, int i, float curr_limit[7])
{

	/* -------------------------------- DECLARATION --------------------------- */
	float vr_TsMis;
	float vr_FiltCurDyn_Freq;
	float vr_FiltCurMis_Freq;

	float vr_FiltCurDyn_a1;
	float vr_FiltCurDyn_a2;
	float vr_FiltCurDyn_ExpCoef ;
	float vr_FiltCurDyn_ExpValue;
	float vr_FiltCurMis_a1;
	float vr_FiltCurMis_a2;
	float vr_FiltCurMis_ExpCoef ;
	float vr_FiltCurMis_ExpValue;   

	float vr_FiltCurDyn_1[6];
	float vr_FiltCurDyn_2[6];
	float   vr_CurFiltDyn[6]  ;
	float vr_FiltCurMis_1[6];
	float vr_FiltCurMis_2[6];
	float   vr_CurFiltMis[6]  ;
	float       vr_CurRes[6];   
	float     vr_IstError[6];
	//bool  vb_ContFlag;
	bool  vb_IstFlag;
	bool  vb_DoubleCheckFlag;
	bool  vb_CollisionFlag;
	float vr_Kf_Thrs_Ist;
	float vr_Kf_Thrs_Diff;
	
    
    /* -------------------------------- SETTINGS --------------------------- */
    /* Sample Time */
      vr_TsMis = 0.010f;

    /* Cur Dyn Freq */
      vr_FiltCurDyn_Freq = 0.78f; // definito in questa funzione
      
    /* Cur Mis Freq */
      vr_FiltCurMis_Freq = 2.0f; // definito in questa funzione
    
	/* CUSTOMIZZAZIONE SOGLIE    */
	if(curr_limit[i] < 0)
	{
		vr_Kf_Thrs_Ist = MAX_THRS;  
	}
	else
	{
		vr_Kf_Thrs_Ist = curr_limit[i] * CollisionFactor;
	}
	vr_Kf_Thrs_Diff = curr_limit[6]; // set the threshold for the difference between current residuals for the double check
	
	/* IMPLEMETAZIONE DEI FILTRI    */
	
	/* Implementazione filtro su corrente modello dinamico       */   
	if (vr_FiltCurDyn_Freq <= QUASIZERO_FLOAT)
	{
	  vr_FiltCurDyn_a1 = ZERO_FLOAT;
	  vr_FiltCurDyn_a2 = ZERO_FLOAT;

	}
	else
	{
	  vr_FiltCurDyn_ExpCoef  =     - TWO_FLOAT * PI_GRECO * vr_FiltCurDyn_Freq * vr_TsMis;
	  vr_FiltCurDyn_ExpValue =                          vr_FiltCurDyn_ExpCoef/4.0 + 1.0;
	  vr_FiltCurDyn_ExpValue = vr_FiltCurDyn_ExpValue * vr_FiltCurDyn_ExpCoef/3.0 + 1.0;
	  vr_FiltCurDyn_ExpValue = vr_FiltCurDyn_ExpValue * vr_FiltCurDyn_ExpCoef/2.0 + 1.0;
	  vr_FiltCurDyn_ExpValue = vr_FiltCurDyn_ExpValue * vr_FiltCurDyn_ExpCoef/1.0 + 1.0;
	  vr_FiltCurDyn_a1       =                        - TWO_FLOAT * vr_FiltCurDyn_ExpValue;
	  vr_FiltCurDyn_a2       =          vr_FiltCurDyn_ExpValue * vr_FiltCurDyn_ExpValue; 
	}
      
   /* Implementazione filtro su corrente misurata */
       
	if (vr_FiltCurMis_Freq <= QUASIZERO_FLOAT)
	{
	  vr_FiltCurMis_a1 = ZERO_FLOAT;
	  vr_FiltCurMis_a2 = ZERO_FLOAT;

	}
	else
	{          
	  vr_FiltCurMis_ExpCoef  =     - TWO_FLOAT * PI_GRECO * vr_FiltCurMis_Freq * vr_TsMis;
	  vr_FiltCurMis_ExpValue =                          vr_FiltCurMis_ExpCoef/4.0 + 1.0;
	  vr_FiltCurMis_ExpValue = vr_FiltCurMis_ExpValue * vr_FiltCurMis_ExpCoef/3.0 + 1.0;
	  vr_FiltCurMis_ExpValue = vr_FiltCurMis_ExpValue * vr_FiltCurMis_ExpCoef/2.0 + 1.0;
	  vr_FiltCurMis_ExpValue = vr_FiltCurMis_ExpValue * vr_FiltCurMis_ExpCoef/1.0 + 1.0;
	  vr_FiltCurMis_a1       =                        - TWO_FLOAT * vr_FiltCurMis_ExpValue;
	  vr_FiltCurMis_a2       =          vr_FiltCurMis_ExpValue * vr_FiltCurMis_ExpValue;          
	}

	_curfilt_cnt[i] ++;
	if (_curfilt_cnt[i] == 1)
	{
	 /* 1st time: */ 
	  _FiltCurDyn_Regr2[i]  = vr_CurDyn;
	  _FiltCurDyn_Regr1[i]  = vr_CurDyn;
	  vr_FiltCurDyn_1[i]    = vr_CurDyn;
	  vr_FiltCurDyn_2[i]    = vr_CurDyn;
	  vr_CurFiltDyn[i]      = vr_CurDyn; 

	  _FiltCurMis_Regr2[i]  = vr_CurMis;
	  _FiltCurMis_Regr1[i]  = vr_CurMis;
	  vr_FiltCurMis_1[i]    = vr_CurMis;
	  vr_FiltCurMis_2[i]    = vr_CurMis;
	  vr_CurFiltMis[i]      = vr_CurMis;   

		// palSetPad(GPIOA,11);
		// palSetPad(GPIOA,12);
	}
	else
	{
	  /* next */ //a1 a2 ok, regr1 regr2 1 2 no!
	   
	 vr_FiltCurDyn_1[i]   = _FiltCurDyn_Regr1[i];
	 vr_FiltCurDyn_2[i]   = _FiltCurDyn_Regr2[i];
	 vr_CurFiltDyn[i]     = (ONE_FLOAT + vr_FiltCurDyn_a1 + vr_FiltCurDyn_a2) * vr_CurDyn - vr_FiltCurDyn_a1 * vr_FiltCurDyn_1[i] - vr_FiltCurDyn_a2 * vr_FiltCurDyn_2[i];
	 _FiltCurDyn_Regr2[i] = _FiltCurDyn_Regr1[i];
	 _FiltCurDyn_Regr1[i] = vr_CurFiltDyn[i]; 


	 vr_FiltCurMis_1[i]   = _FiltCurMis_Regr1[i];
	 vr_FiltCurMis_2[i]   = _FiltCurMis_Regr2[i];
	 vr_CurFiltMis[i]     = (ONE_FLOAT + vr_FiltCurMis_a1 + vr_FiltCurMis_a2) * vr_CurMis - vr_FiltCurMis_a1 * vr_FiltCurMis_1[i] - vr_FiltCurMis_a2 * vr_FiltCurMis_2[i];
	 _FiltCurMis_Regr2[i] = _FiltCurMis_Regr1[i];
	 _FiltCurMis_Regr1[i] = vr_CurFiltMis[i];
	
	}
        
    /* CALCOLO RESIDUI */  
	vr_CurRes[i]     = vr_CurFiltDyn[i] - vr_CurFiltMis[i];
	_cur_res_rasp[i]     = vr_CurRes[i] ;
	joints_control_.joints[i].R_rasp = vr_CurRes[i] ;
	vr_IstError[i]   = fabs(vr_CurRes[i]);    
		
    /* APPLICAZIONE DELLE SOGLIE */
    
    // soglia su valore istantaneo 
    if (vr_IstError[i] > vr_Kf_Thrs_Ist)  
    {
      vb_IstFlag = true; 
      //printf("CurRes: %f \t Jnt: %d\n", vr_IstError[i], i+1);
    }
    else
    {
      vb_IstFlag = false;           
    }
	
    // soglia sul valore continuo
    //vb_ContFlag = false;
		
    // soglia su differenza resdui
    if ( (fabs(_cur_res_rasp[i]-_cur_res_joint[i]) > vr_Kf_Thrs_Diff) && (bc_flag==false) )  
    {
      vb_DoubleCheckFlag = true; 
      //printf("DoubleCheckRes: %f \t Jnt: %d\n", fabs(_cur_res_rasp[i]-_cur_res_joint[i]), i+1);
    }
    else
    {
      vb_DoubleCheckFlag = false;           
    }
	 
    // OR sulle soglie
    if (vb_DoubleCheckFlag || vb_IstFlag)
    {  
      vb_CollisionFlag = true;
    }
    else
    {
      vb_CollisionFlag = false;  
    }  
    
  return vb_CollisionFlag; 

} // AlgorithmManager::collisionCheck

// Message to set and customize threshold on the raspberry side
void AlgorithmManager::algoCollisionThr(edo_core_msgs::CollisionThreshold msg)
{
	for(int i = 0; i < 6; ++i)
	{
		if(msg.joints_mask & 1<<i)
		{
			curr_limit[i]=msg.threshold;
		}
	}
	
	if(msg.joints_mask & 1<<6)
	{
		curr_limit[6]=msg.threshold;
	}
}

// If in brakes check state the collision threshols are reduced by this CollisionFactor
void AlgorithmManager::machineStateCallback(edo_core_msgs::MachineState msg)
{
	if(msg.current_state == 7 || msg.current_state == 8)
	{
		bc_flag = true;
		CollisionFactor = 0.4;
	}
	else
	{
		bc_flag = false;
		CollisionFactor = 1;
	}
}

void AlgorithmManager::jntConfigCallback(edo_core_msgs::JointConfigurationArray msg)
{
	curr_limit[0] = msg.joints[0].kt + 0.1;
	curr_limit[1] = msg.joints[1].kt + 0.1;
	curr_limit[2] = msg.joints[2].kt + 0.1;
	curr_limit[3] = msg.joints[3].kt + 0.1;
	curr_limit[4] = msg.joints[4].kt + 0.1;
	curr_limit[5] = msg.joints[5].kt + 0.1;
}

void AlgorithmManager::dynamicModelReset()
{
	(void)ORL_cancel_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	memcpy(&hold_position_, &current_state_, sizeof(ORL_joint_value));
	first_time_ = true;
	while (!MoveCommandMsgListInOrl_.empty())
	{
		MoveCommandMsgListInOrl_.pop_front();
		feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 30, __FUNCTION__, __LINE__); 
	}        
	pause_state_ == false;
	pending_cancel_ = false;
	waiting_ = false;
	setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__);
	
	/*  SET POS  */
	#if ENABLE_DYNAMIC_MODEL_PRINTFS
	printf("[DYM] current %g,%g,%g,%g,%g,%g\n",sx_dyn_mod_.current [ORL_AX1],sx_dyn_mod_.current [ORL_AX2],sx_dyn_mod_.current [ORL_AX3],sx_dyn_mod_.current [ORL_AX4],sx_dyn_mod_.current [ORL_AX5],sx_dyn_mod_.current [ORL_AX6]);
	#endif
	
	ORL_joint_value update_dyn_;
	for(size_t j = 0; j < joints_control_.size; j++)
	{
		update_dyn_.value[j] = joints_control_.joints[j].position;
	}
	
	ORL_set_position(NULL, &update_dyn_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	
	ORL_Dynamic_Model sx_dyn_mod_fake;
	memset(&sx_dyn_mod_fake, 0, sizeof(ORL_Dynamic_Model));
	
	for(size_t i = 0; i < NR_GET_NEXT_STEP_CALLS ; i++)
	{
	    ORL_get_dynamic_model(NULL, NULL, NULL, &sx_dyn_mod_fake, 1, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	    #if ENABLE_DYNAMIC_MODEL_PRINTFS
	    printf("[DYM_ALARM_fake] current %g,%g,%g,%g,%g,%g\n",sx_dyn_mod_fake.current [ORL_AX1],sx_dyn_mod_fake.current [ORL_AX2],sx_dyn_mod_fake.current [ORL_AX3],sx_dyn_mod_fake.current [ORL_AX4],sx_dyn_mod_fake.current [ORL_AX5],sx_dyn_mod_fake.current [ORL_AX6]);
	    printf("[DYM_ALARM_real] current %g,%g,%g,%g,%g,%g\n",sx_dyn_mod_.current [ORL_AX1],sx_dyn_mod_.current [ORL_AX2],sx_dyn_mod_.current [ORL_AX3],sx_dyn_mod_.current [ORL_AX4],sx_dyn_mod_.current [ORL_AX5],sx_dyn_mod_.current [ORL_AX6]);
	    #endif
	}
	
	ORL_get_dynamic_model(NULL, NULL, NULL, &sx_dyn_mod_, 1, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	//#if ENABLE_DYNAMIC_MODEL_PRINTFS
	printf("[DYM_ALARM] current %g,%g,%g,%g,%g,%g\n",sx_dyn_mod_.current [ORL_AX1],sx_dyn_mod_.current [ORL_AX2],sx_dyn_mod_.current [ORL_AX3],sx_dyn_mod_.current [ORL_AX4],sx_dyn_mod_.current [ORL_AX5],sx_dyn_mod_.current [ORL_AX6]);
	//#endif
	
	for(size_t i = 0; i < joints_control_.size; i++)
	{
		hold_current_.current[i] = sx_dyn_mod_.current[i];
	}
	alarm_state_ack_ = true;  
}

void AlgorithmManager::jntResetCallback(edo_core_msgs::JointResetConstPtr msg)
{
	collTimer.start();
	_collision_disable = true;
}

void AlgorithmManager::unbrakeTimerCallback(const ros::TimerEvent& event)
{
	collTimer.stop();
	_collision_disable = false;
}