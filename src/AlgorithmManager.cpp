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
#define ENABLE_ROS_WARN  (1==1)
#define ENABLE_ROS_INFO  (1==1)
#define ENABLE_ALGOMNGR_PRINTFS (1==1)
#define ENABLE_MINIMAL_ALGOMNGR_PRINTFS (1==1)
#else
/**this sets verbosity level of ORL library**/
#define LOCAL_VERBOSITY ORL_SILENT /**** ORL_VERBOSE or **ORL_SILENT**/
#define ENABLE_ROS_WARN  (1==0)
#define ENABLE_ROS_INFO  (1==0)
#define ENABLE_ALGOMNGR_PRINTFS (1==0)
#define ENABLE_MINIMAL_ALGOMNGR_PRINTFS (1==0)
#endif  

#define DO_TERMINATE_AFTER_CANCEL (1==0)
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

	feedback_publisher_ = node.advertise<edo_core_msgs::MovementFeedback>("algo_movement_ack", 200);
	cartesian_pose_pub_ = node.advertise<edo_core_msgs::CartesianPose>("cartesian_pose", 100);
	algorithm_state_pub_ = node.advertise<std_msgs::Int8>("algorithm_state", 100);

	//create a ROS Service Server
	ros::ServiceServer get_jnts_number_srv = node.advertiseService("algo_jnt_number_srv", &AlgorithmManager::getJointsNumber, this);
#if ENABLE_KINEMATICS_SERVICES
	ros::ServiceServer get_direct_kinematics_srv = node.advertiseService("algo_direct_kinematics_srv", &AlgorithmManager::getDirectKinematics, this);
	ros::ServiceServer get_inverse_kinematics_srv = node.advertiseService("algo_inverse_kinematics_srv", &AlgorithmManager::getInverseKinematics, this);
#endif
	ros::ServiceServer loadConfigurationFile_srv = node.advertiseService("algo_load_configuration_file_srv", &AlgorithmManager::loadConfigurationFile_CB, this);
	robot_switch_control_server = node.advertiseService("algo_control_switch_srv", &AlgorithmManager::SwitchControl, this);

	//publish /jnt_ctrl to rosserial, then joints
	ros::Publisher robot_control_publisher = node.advertise<edo_core_msgs::JointControlArray>("algo_jnt_ctrl", 100);
	
	//memset(&next_move_request_, 0, sizeof(edo_core_msgs::MovementFeedback));
	memset(&target_joint_, 0, sizeof(ORL_joint_value));
	memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
	memset(&hold_position_, 0, sizeof(ORL_joint_value));
	memset(&current_state_, 0, sizeof(ORL_joint_value));
	delay_ = 255;
	waiting_ = false;
	jog_state_ = false;
	pending_cancel_ = false;
	jog_carlin_state_ = false;
	pause_state_ = false;
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

	private_nh.param<double>("controller_frequency", controller_frequency_, CONTROLLER_FREQUENCY);
	private_nh.param<double>("state_saturation_threshold", state_saturation_threshold_, 0.5);
	private_nh.param<int>("interpolation_time_step", interpolation_time_step_, INTERPOLATION_STEP);

	// Duration, callback, callback-owner, oneshot, autostart
	timerCalib_ = private_nh.createTimer(ros::Duration(0.5), &AlgorithmManager::timerCallback, this, true, false);
	
	ros::Rate loop_rate(controller_frequency_);
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
	last_publish_time = ros::Time::now();
#endif
	//Spin asincrono
	ros::AsyncSpinner aspin(2);
	aspin.start();

	while (ros::ok())
	{
		if (algorithm_mode_ != SWITCHED_OFF){
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
  if ((algorithm_mode_ != UNINITIALIZED) &&
      (configurationFileLoaded_ == 1))
  {
	  ORL_cancel_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	  ORL_terminate_controller(LOCAL_VERBOSITY, ORL_CNTRL01);
  }
}

void AlgorithmManager::setAlgorithmMode(AlgorithmManager::Mode si_mode, const char *apc_func, int si_line)
{
static const char *sam_AlgorithmMode[MAX_NUM_ALGORITHM_MODES] = {
		"UNINITIALIZED(0)",
		"INITIALIZED(1)",
		"MOVING(2)",
		"WAITING(3)",
		"BLOCKED(4)",
		"FINISHED(5)",
		"PAUSE(6)",
		"RECOVERY(7)",
		"SWITCHED_OFF(8)"
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
	int status = ORL_initialize_controller("robot.edo", (pkg_path_ + "/config/").c_str(), LOCAL_VERBOSITY, ORL_CNTRL01); //(pkg_path_ + "/config/").c_str()
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
		ORL_System_Variable     orl_sys_var;
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

  // printf("[%s,%d] loadConfigurationFile_CB\n",__FILE__,__LINE__);
  if ((algorithm_mode_ == UNINITIALIZED) &&
      (configurationFileLoaded_ == 0)) // Configuration file already loaded?
  { /* No */
    sb_sts = AlgorithmManager::initializeORL();
    if (sb_sts == false)
    {
      ROS_ERROR("Failure loading configuration file");
      //printf("[%s,%d] ROS_ERROR Failure loading configuration file\n",__FILE__,__LINE__);
    }
    else 
    {
      configurationFileLoaded_ = 1;  // Configuration file loaded

      edo_core_msgs::JointsNumber::Request reqgjn;
      edo_core_msgs::JointsNumber::Response resgjn;

      joints_number_ = 0;

      if(!getJointsNumber(reqgjn, resgjn))
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
    printf("[%s,%d] algorithm_mode_ %d configurationFileLoaded_ %d\n",__FILE__,__LINE__, algorithm_mode_, configurationFileLoaded_);
  }
#endif
  res.result = sb_sts;
  return(sb_sts);
}

bool AlgorithmManager::getJointsNumber(edo_core_msgs::JointsNumber::Request  &req, edo_core_msgs::JointsNumber::Response &res){

  // printf("[%s,%d] getJointsNumber\n",__FILE__,__LINE__);
  if (configurationFileLoaded_ == 1)  // Configuration file loaded
  {
    ORL_System_Variable     orl_sys_var;

    sprintf(orl_sys_var.sysvar_name, "$ARM_DATA[%d].RRS_TOL_CTIME", ORL_ARM1 + 1); 
    orl_sys_var.ctype = ORL_INT; //  ORL_BOOL  ORL_REAL  ORL_STRING
    orl_sys_var.iv = 0;
    ORL_set_data   (orl_sys_var,      /* [IN]      Structure for the system variable */
        LOCAL_VERBOSITY , /* [IN]      Verbose ON/OFF */
        ORL_CNTRL01       /* [IN]      Controller Index */);
    sprintf(orl_sys_var.sysvar_name, "$ARM_DATA[%d].RRS_TOL_FTIME", ORL_ARM1 + 1); 
    orl_sys_var.ctype = ORL_INT; //  ORL_BOOL  ORL_REAL  ORL_STRING
    orl_sys_var.iv = 0;
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
printf("[%s,%d] ORL_get_sys_var.$AUX_MASK failure\n",__FILE__,__LINE__);
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
printf("[%s,%d] ORL_get_sys_var.$JNT_MASK failure\n",__FILE__,__LINE__);
      return true;
    }
    if (jnt_mask <= 0)
    {
      res.counter = -3;
printf("[%s,%d] getJointsNumber\n",__FILE__,__LINE__);
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
    
    if ((jnt_num != 4) && (jnt_num != 6) && (jnt_num != 7))
printf("[%s,%d] getJointsNumber Jnt_Mask:%d Aux_Mask:%d Axes:%d\n",__FILE__,__LINE__, jnt_mask_save, aux_mask, jnt_num);

    res.counter = jnt_num;
    res.joints_mask = (unsigned long)jnt_mask_save;
    res.joints_aux_mask = (unsigned long)aux_mask;
    // printf("[%s,%d] getJointsNumber Jnt_Mask:%d Aux_Mask:%d Axes:%d\n",__FILE__,__LINE__, jnt_mask_save, aux_mask, jnt_num);
  }
  else
  {
    res.counter = -2;
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

  if(algorithm_mode_!=UNINITIALIZED)
  {
    feedbackFn(MESSAGE_FEEDBACK::COMMAND_RECEIVED, 0, __FUNCTION__, __LINE__);

#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
printf ("[moveCallback,%d] MOVE_MESSAGE_COMMAND = %c algorithm_mode_ %d\n",__LINE__,msg->move_command, algorithm_mode_);
#endif

#if ENABLE_ROS_INFO
    ROS_INFO("MOVE_MESSAGE_COMMAND = %c command received...   sono nello stato %d", msg->move_command, algorithm_mode_);
#endif
    //gestisco il delay, settato al controllo precedente
    //se 255 passo oltre (no_delay), altrimenti aspetto
    if (pause_state_                                                 &&
        (msg->move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_RESUME) &&
        (msg->move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL) &&
        (msg->move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE))
    {
      ROS_ERROR("Edo is in pause state! Please resume or cancel previous movement...");
      return;
    }
    
    ros::Rate loop_rate(controller_frequency_);
    while(algorithm_mode_ == WAITING || waiting_ )
    {
      if (!ros::ok())
      {
        return;
      }
      loop_rate.sleep();
    }
    
    //qui leggo il delay
    if((msg->move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE)  && 
       (msg->move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_RESUME) && 
       (msg->move_command != E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL))
    {
      delay_ = msg->delay;
      if (msg->delay != 255 && msg->delay != 0) 
      {
        waiting_ = true;
      }
    }
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
		feedbackFn(MESSAGE_FEEDBACK::COMMAND_RECEIVED, 0, __FUNCTION__, __LINE__);
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
	else{
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
	if (msg->joints.size() != joints_number_) {
		ROS_WARN_THROTTLE(10, "Unacceptable state, impossible to initialize ORL...");
		return;
	}
#if ENABLE_ROS_WARN
	//ROS_WARN("dati %d, %d, %d, %d", strk[0][0], strk[1][0], strk[0][1], strk[1][1]);
#endif
	current_state_.unit_type = ORL_POSITION_LINK_DEGREE;
	for(size_t i = 0; i < msg->joints.size(); i++)
	{
		if (msg->joints[i].position <= (float)strk_[0][i]) {
			if (fabs(msg->joints[i].position - (float)strk_[0][i]) <= state_saturation_threshold_) {
				current_state_.value[i] = (float)strk_[0][i];
				ROS_WARN_THROTTLE(2, "joint %d: position saturated N", i+1);
			}
			else
			{
				ROS_ERROR("Error, joint %d: stroke end N", i+1);
				//algorithm_mode_ = BLOCKED;//INITIALIZED;
				current_state_.value[i] = (float)strk_[0][i];
				hold_position_.value[i] = current_state_.value[i];
				//memcpy(&hold_position_, &current_state_, sizeof(ORL_joint_value));
			}
		}
		else if (msg->joints[i].position >= (float)strk_[1][i]) {
			if (fabs(msg->joints[i].position - (float)strk_[1][i]) <= state_saturation_threshold_) {
				current_state_.value[i] = (float)strk_[1][i];
				ROS_WARN_THROTTLE(2, "joint %d: position saturated P", i+1);
			}
			else
			{
				ROS_ERROR("Error, joint %d: stroke end P", i+1);
				//algorithm_mode_ = BLOCKED;//INITIALIZED;
				current_state_.value[i] = (float)strk_[1][i];
				hold_position_.value[i] = current_state_.value[i];
				//memcpy(&hold_position_, &current_state_, sizeof(ORL_joint_value));
			}
		}
		else
		{
			current_state_.value[i] = msg->joints[i].position;
		}
	}
  if ((algorithm_mode_ == UNINITIALIZED) &&
      (configurationFileLoaded_ == 1) // Configuration file already loaded? Yes
	   )
		initialize(msg);
       
  if (algorithm_mode_ != UNINITIALIZED)
  {
    if (++aggPosCartPose_ >= 10)
    {
    	  // every ~100ms compute the cartesian position
	  	  cartesian_pose_pub_.publish(computeCartesianPose(&current_state_));
	  	  aggPosCartPose_ = 0;
    }
  }
}

static void frame_dump(const char *name, int line, ORL_cartesian_position cp);
static void frame_dump(const char *name, int line, ORL_cartesian_position cp)
{
#if ENABLE_ROS_INFO
  ROS_INFO("[%s,%d] ", name, line, cp.x);
  ROS_INFO("[%s,%d] ", name, line, cp.y);
  ROS_INFO("[%s,%d] ", name, line, cp.z);
#endif
  return;
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
        target_joint_.value[i] = msg->target.joints_data[i];
// printf("[moveTrjntFn,%d] Joint %d moving from %f to %f\n",__LINE__,i,current_state_.value[i],target_joint_.value[i]);
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
    // MOVE JOINT TO <cartesian_position>
    memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
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
        target_joint_.value[i] = 0.0;
      }
    }
    memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
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
    first_time_ = false;
  }

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
    /*                             $BASE     $TOOL       $UFRAME                                          */
    status = ORL_initialize_frames (base_, tool_local, uframe_local, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    if(!manageORLStatus(status,"ORL_initialize_frames"))
    {
      return false;
    }
  }
  
  {
    std::vector<int> move_parameters;
    move_parameters.resize(3);
    
    int status = 0;
    if (delay_ == 255) 
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
    memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
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
    memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
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
    first_time_ = false;
  }
  
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
    frame_dump("FRAME", __LINE__, uframe_local);
    /*                             $BASE     $TOOL       $UFRAME                                          */
    status = ORL_initialize_frames (base_, tool_local, uframe_local, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    if(!manageORLStatus(status,"ORL_initialize_frames"))
    {
      return false;
    }
  }
  
  {
    std::vector<int> move_parameters;
    move_parameters.resize(3);
    
    if (delay_ == 255) {
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
#if DEVELOPMENT_RELEASE
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
    memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
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
    memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
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
    first_time_ = false;
  }
  
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
    uframe_local.unit_type = ORL_CART_POSITION;
    uframe_local.x = msg->frame.x;
    uframe_local.y = msg->frame.y;
    uframe_local.z = msg->frame.z;
    uframe_local.a = msg->frame.a;
    uframe_local.e = msg->frame.e;
    uframe_local.r = msg->frame.r;
    /*                             $BASE     $TOOL       $UFRAME                                          */
    status = ORL_initialize_frames (base_, tool_local, uframe_local, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    if(!manageORLStatus(status,"ORL_initialize_frames"))
    {
      return false;
    }
  }

  {  
    std::vector<int> move_parameters;
    
    move_parameters.resize(3);
    if (delay_ == 255) {
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
  
    status = setORLMovement(move_parameters, msg->move_type, msg->via.data_type, msg->ovr);
    if(!manageORLStatus(status,"setORLMovement"))
    {
      return false;
    }
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
    current_state_local.value[i]=std::max(strk_[0][i]+0.1, std::min(strk_[1][i]-0.1, current_state_.value[i]));
  }
  
  memcpy(&target_joint_, &current_state_local, sizeof(ORL_joint_value));
  target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
  {
    unsigned int sj_lastJointAxis;
    unsigned long sm_joints_mask;
    target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
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
          target_joint_.value[i] = (msg->target.joints_data[i] < 0) ? strk_[0][i] + 0.1 : strk_[1][i] - 0.1;
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

  status = ORL_set_position(NULL, &current_state_local, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
  if(!manageORLStatus(status,"ORL_set_position"))
  {
    return false;
  }
  
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
    memset(&uframe_local, 0, sizeof(ORL_cartesian_position));
    uframe_local.unit_type = ORL_CART_POSITION;
    uframe_local.x = msg->frame.x;
    uframe_local.y = msg->frame.y;
    uframe_local.z = msg->frame.z;
    uframe_local.a = msg->frame.a;
    uframe_local.e = msg->frame.e;
    uframe_local.r = msg->frame.r;
    /*                             $BASE     $TOOL       $UFRAME                                          */
    status = ORL_initialize_frames (base_, tool_local, uframe_local, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    if(!manageORLStatus(status,"ORL_initialize_frames"))
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
printf ("[movePauseFn,%d] B PAUSE Move algorithm_mode_ %d\n",__LINE__,algorithm_mode_);
#endif
  if(algorithm_mode_ == MOVING)
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
  }
  else
  {
    setAlgorithmMode(PAUSE, __FUNCTION__,__LINE__);
  }
  
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[movePauseFn,%d] E PAUSE Move algorithm_mode_ %d\n",__LINE__,algorithm_mode_);
#endif
  feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 0, __FUNCTION__, __LINE__);	
  return true;
}

bool AlgorithmManager::moveResumeFn(edo_core_msgs::MovementCommand * msg)
{  
  int status;
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[%s,%d] B RESUME Move algorithm_mode_ %d\n",__FILE__,__LINE__,algorithm_mode_);
#endif
  if (algorithm_mode_ == PAUSE)
  {
    //int status = ORL_resume_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
#if ENABLE_ROS_WARN
    ROS_WARN("enter moveResumeFn");
#endif
    setAlgorithmMode(RECOVERY, __FUNCTION__,__LINE__);
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[moveResumeFn,%d] ORL_cancel_motion algorithm_mode_ %d\n",__LINE__,algorithm_mode_);
#endif
    status = ORL_cancel_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
#if DO_TERMINATE_AFTER_CANCEL
#if ENABLE_ROS_WARN
		ROS_WARN("Terminate and re-initialize ORL...");
#endif

    pthread_mutex_lock(&control_mutex_);
    
    status = ORL_terminate_controller(LOCAL_VERBOSITY, ORL_CNTRL01);
    if (!initializeORL())
    {
      ROS_ERROR("Impossible to Inizialize ORL");
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[moveResumeFn,%d] Failure initializeORL\n",__LINE__);
#endif
      return false;
    }
    
    pthread_mutex_unlock(&control_mutex_);
    
    hold_position_.unit_type = ORL_POSITION_LINK_DEGREE; //TODO aggio il problema ma chi cancella il tipo, in caso di jog cartesiano?
    status = ORL_set_position(NULL, &hold_position_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    if(!manageORLStatus(status,"ORL_set_position"))
    {
      return false;
    }
#endif
    
    if (in_progress_moveCommand_.move_type == E_MOVE_TYPE::E_MOVE_TYPE_JOINT)
    {
      // MOVE JOINT  TO <xxxx>
      if(!moveTrjntFn(&in_progress_moveCommand_))
      {
        ROS_ERROR("Move Joint not executed...");
        feedbackFn(MESSAGE_FEEDBACK::ERROR, State::FAILED_MOVE, __FUNCTION__, __LINE__);
      }
    }
    else if (in_progress_moveCommand_.move_type == E_MOVE_TYPE::E_MOVE_TYPE_LINEAR)
    { 
      // MOVE LINEAR TO <xxxx>
      if(!moveCarlinFn(&in_progress_moveCommand_))
      {
        ROS_ERROR("Move Linear not executed...");
        feedbackFn(MESSAGE_FEEDBACK::ERROR, State::FAILED_MOVE, __FUNCTION__, __LINE__);
      }
    }
    else if (in_progress_moveCommand_.move_type == E_MOVE_TYPE::E_MOVE_TYPE_CIRCULAR)
    {
      // MOVE CIRCULAR TO <xxxx>
      if(!moveCarcirFn(&in_progress_moveCommand_))
      {
        ROS_ERROR("Move Circular not executed...");
        feedbackFn(MESSAGE_FEEDBACK::ERROR, State::FAILED_MOVE, __FUNCTION__, __LINE__);
      }
    }
    // TODO all the other combination with XTND_POSITION
    pause_state_ = false;
    setAlgorithmMode(MOVING, __FUNCTION__,__LINE__);
  }
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[moveResumeFn,%d] E RESUME Move algorithm_mode_ %d\n",__LINE__,algorithm_mode_);
#endif  
  feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 0, __FUNCTION__, __LINE__);
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
#if DO_TERMINATE_AFTER_CANCEL
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[moveCancelFn,%d] ORL_terminate_controller\n",__LINE__);
#endif
#if ENABLE_ROS_WARN
		ROS_WARN("Terminate and re-initialize ORL...");
#endif

		pthread_mutex_lock(&control_mutex_);
		
		status = ORL_terminate_controller(LOCAL_VERBOSITY, ORL_CNTRL01);
		if (!initializeORL())
		{
			ROS_ERROR("Impossible to Inizialize ORL");
printf ("[moveCancelFn,%d] Failure initializeORL\n",__LINE__);
		}
		
		pthread_mutex_unlock(&control_mutex_);
		
		hold_position_.unit_type = ORL_POSITION_LINK_DEGREE; //TODO aggio il problema ma chi cancella il tipo, in caso di jog cartesiano?
		status = ORL_set_position(NULL, &hold_position_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
#endif
		setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__); //finished????
		pause_state_ = false;
		pending_cancel_ = false;
		waiting_ = false;
	}
	else if (algorithm_mode_ == WAITING)
	{
		// Request to CANCEL while waiting for the delay expiration
		setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__);
		delay_ = 255;
	}
	else
	{
		// If I can't satisfy the request now, I remember it.
		pending_cancel_ = true;
	}
	
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[moveCancelFn,%d] E Cancel Move algorithm_mode_ %d\n",__LINE__,algorithm_mode_);
#endif
	feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 0, __FUNCTION__, __LINE__);
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
    current_state_local.value[i] = std::max(strk_[0][i]+0.1, std::min(strk_[1][i]-0.1, current_state_.value[i]));
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
          target_joint_.value[i] = current_state_.value[i];
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
            target_joint_.value[i] = (msg->target.joints_data[i] < 0) ? strk_[0][i] + 0.1 : strk_[1][i] - 0.1;
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
    /*                             $BASE     $TOOL       $UFRAME                                          */
    status = ORL_initialize_frames (base_, tool_local, uframe_local, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
    if(!manageORLStatus(status,"ORL_initialize_frames"))
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
    
  status = ORL_set_position(NULL, &current_state_local, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);  
  if(!manageORLStatus(status,"ORL_set_position"))
  {
    return false;
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

//funzione di jog in spazio cartesiano
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
#endif
	if (move_parameters.size() != 3) 
	{
		ROS_ERROR("Invalid parameters...");
		return MOVETYPE_UNDEF;
	}
	
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
    si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARCIR, NULL, &target_joint_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
  }
  else if ((movement_type == E_MOVE_TYPE::E_MOVE_TYPE_CIRCULAR) && (point_data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_POSITION))
  { // MOVE CIRCULAR TO <CART_POSITION> VIA <CART_POSITION>
#if ENABLE_ROS_INFO
    ROS_INFO("MOVE CIRCULAR TO <CART_POSITION> VIA <CART_POSITION>");
#endif
    si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARCIR, &target_cart_, NULL, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
  }
  else if ((movement_type == E_MOVE_TYPE::E_MOVE_TYPE_CIRCULAR) && (point_data_type == E_MOVE_DEST_POINT::E_MOVE_POINT_XTND_POS))
  { // MOVE CIRCULAR TO <XTND_POSITION> VIA <XTND_POSITION>
#if ENABLE_ROS_INFO
    ROS_INFO("MOVE CIRCULAR TO <XTND_POSITION> VIA <XTND_POSITION>");
#endif
    si_status = ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARCIR, &target_cart_, &target_joint_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
  }
  else
  {
    ROS_ERROR("MOVE UNDEFINED");
    si_status = MOVETYPE_UNDEF; //Movement type undefined
  }
#if ENABLE_ALGOMNGR_PRINTFS
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
		joints_control_.joints[i].current = 0.0;
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
		setAlgorithmMode(BLOCKED, __FUNCTION__,__LINE__);
		memcpy(&hold_position_, &current_state_, sizeof(ORL_joint_value));
		feedbackFn(MESSAGE_FEEDBACK::ERROR, status, __FUNCTION__, __LINE__);
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
  static bool debug1 = true;
  static bool debug2 = true;
  std_msgs::Int8 temp_algo_state;
  temp_algo_state.data = algorithm_mode_;
  algorithm_state_pub_.publish(temp_algo_state);
  edo_core_msgs::MovementCommand msg;
  
  //scodo i messaggi ricevuti per le move 
  while (!msglist_.empty())
  {
#if ENABLE_ROS_INFO
    ROS_INFO("pop move command");
#endif
    msg=msglist_.back();
    msglist_.pop_back();
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
printf ("[updateControl,%d] GET item %c\n",__LINE__,msg.move_command);
#endif
    if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_MOVE)
    {
      in_progress_moveCommand_ = msg;  // save the move command in progress, can be used later for the resume
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
    else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_PAUSE)
    {
      if(!movePauseFn(&msg))
      {
        ROS_ERROR("Pause not executed...");
        feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR, __FUNCTION__, __LINE__);
      }
    }
    else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_RESUME)
    {
      if(!moveResumeFn(&msg))
      {
        ROS_ERROR("Resume not executed...");
        feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR, __FUNCTION__, __LINE__);
      }
    }
    else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_CANCEL)
    {
      if(!moveCancelFn(&msg))
      {
        ROS_ERROR("Cancel not executed...");
        feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR, __FUNCTION__, __LINE__);
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
    msglistJog_.pop_back();

    if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_JOGMOVE)
    {
      if (msg.move_type == E_MOVE_TYPE::E_MOVE_TYPE_JOINT)
      {
        if(!jogTrjntFn(&msg))
        {
          ROS_ERROR("Move JNTtrj not executed...");
          feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR, __FUNCTION__, __LINE__);
        }
      }
      else if (msg.move_type == E_MOVE_TYPE::E_MOVE_TYPE_LINEAR)
      {
        if(!jogCarlinFn(&msg))
        {
          ROS_ERROR("Move CARTtrj not executed...");
          feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR, __FUNCTION__, __LINE__);
        }
      }
    }
    else if (msg.move_command == E_MOVE_COMMAND::E_MOVE_COMMAND_JOGSTOP)
    {
      if(!jogStopFn(&msg))
      {
        ROS_ERROR("Move JOG stop not executed...");
        feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR, __FUNCTION__, __LINE__);
      }
    }
    else
    {
      ROS_WARN("Command not implemented, please use only accepted command type");
      feedbackFn(MESSAGE_FEEDBACK::ERROR, State::MOVETYPE_UNDEF, __FUNCTION__, __LINE__);
      return;
    }
  }
  
  if(algorithm_mode_ == MOVING)
  {
    if(debug1)
    {
#if ENABLE_ROS_INFO
      ROS_INFO("ALGO MODULE STARTED SENDING ORL CONTROL");
#endif
      debug1 = false;
      debug2 = true;
    }
    setControl();
  }
  else if(algorithm_mode_ == INITIALIZED)
  {
    if(debug2)
    {
#if ENABLE_ROS_INFO
      ROS_INFO("ALGO MODULE STARTED SENDING KEEP POSITION MESSAGES");
#endif
      debug2 = false;
      debug1 = true;
    }
    keepPosition();
  }
  else if (algorithm_mode_ == WAITING)
  {
    keepPosition();
    ros::Duration d = ros::Time::now() - start_wait_time_;
    if(d.toSec() >= (double)delay_)
    {
      waiting_ = false;
      setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__);
      delay_ = 255;
    }
  }
  else if (algorithm_mode_ == BLOCKED)
  {
    keepPosition();
    waiting_ = false;
    pause_state_ = false;
    pending_cancel_ = false;
    setAlgorithmMode(INITIALIZED, __FUNCTION__,__LINE__);
  }
  else if (algorithm_mode_ == PAUSE)
  {
    int status;
    if (pending_cancel_ == true)
    {
      setAlgorithmMode(RECOVERY, __FUNCTION__,__LINE__);
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[%s,%d] ORL_cancel_motion\n",__FUNCTION__,__LINE__);
#endif
      status = ORL_cancel_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
      (void)manageORLStatus(status,"ORL_cancel_motion"); // don't care the return status
#if DO_TERMINATE_AFTER_CANCEL
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[%s,%d] ORL_terminate_controller\n",__FUNCTION__,__LINE__);
#endif
#if ENABLE_ROS_WARN
      ROS_WARN("Terminate and re-initialize ORL...");
#endif

      pthread_mutex_lock(&control_mutex_);
	  
      status = ORL_terminate_controller(LOCAL_VERBOSITY, ORL_CNTRL01);
      if (!initializeORL())
      {
        ROS_ERROR("Impossible to Inizialize ORL");
        // What can I do now?
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[updateControl,%d] Failure initializeORL\n",__LINE__);
#endif
      }
	  
      pthread_mutex_unlock(&control_mutex_);
	  
      hold_position_.unit_type = ORL_POSITION_LINK_DEGREE; //TODO aggio il problema ma chi cancella il tipo, in caso di jog cartesiano?
      status = ORL_set_position(NULL, &hold_position_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
#endif
      pause_state_ = false;
      pending_cancel_ = false;
      waiting_ = false;
      setAlgorithmMode(INITIALIZED, __FUNCTION__,__LINE__);
    }
    keepPosition();
  }
  else if (algorithm_mode_ == FINISHED)
  {
    int status;

    if (jog_state_ || pending_cancel_)
    {
      setAlgorithmMode(RECOVERY, __FUNCTION__,__LINE__);
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[%s,%d] ORL_cancel_motion\n",__FUNCTION__,__LINE__);
#endif
      status = ORL_cancel_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
      (void)manageORLStatus(status,"ORL_cancel_motion"); // don't care the return status
#if DO_TERMINATE_AFTER_CANCEL
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[%s,%d] ORL_terminate_controller\n",__FUNCTION__,__LINE__);
#endif
#if ENABLE_ROS_WARN
      ROS_WARN("Terminate and re-initialize ORL...");
#endif

      pthread_mutex_lock(&control_mutex_);
	  
      status = ORL_terminate_controller(LOCAL_VERBOSITY, ORL_CNTRL01);
      if (!initializeORL())
      {
        ROS_ERROR("Impossible to Inizialize ORL");
#if ENABLE_ALGOMNGR_PRINTFS
printf ("[updateControl,%d] Failure initializeORL\n",__LINE__);
#endif
        // What can I do now?
      }
	  
      pthread_mutex_unlock(&control_mutex_);
	  
      hold_position_.unit_type = ORL_POSITION_LINK_DEGREE; //TODO aggio il problema ma chi cancella il tipo, in caso di jog cartesiano?
      status = ORL_set_position(NULL, &hold_position_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
#endif
      jog_state_ = false;
      pending_cancel_ = false;
    }

    keepPosition();
    setAlgorithmMode(INITIALIZED, __FUNCTION__,__LINE__);
  }
}

void AlgorithmManager::setControl()
{
	ORL_joint_value burned;
	burned.unit_type = ORL_POSITION_LINK_DEGREE;
	//boost::circular_buffer<ORL_joint_value> interpolation_data(3);
	int status = 0;
	int  sd_started,    
	     sd_stopped,    
	     sd_ended,      
	     sd_decPhase,   
	     sd_flyNode,    
	     sd_flyStarted, 
	     nodo,    
	     nodo_fly,
	     nodo_fly_old = 0;   
	
	for(size_t i = 0; i < NR_GET_NEXT_STEP_CALLS && status != FINAL_STEP; i++)
	{
		numberSteps_++;

		status = ORL_get_next_interpolation_step(&burned, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);

#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
		if ((status == 2) || ((status < 0) && (status != -42)))
		{
				printf("[setControl, ORL_get_next_interpolation_step, %d] numberSteps_ %d numberSteps_M42_ %d status %d\n",__LINE__, numberSteps_, numberSteps_M42_, status);
				numberSteps_ = numberSteps_M42_ = 0;
		}
#endif
    
		if (status == -42)
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
		else if(status == FINAL_STEP)
		{
				if (pause_state_) 
				{
						setAlgorithmMode(PAUSE, __FUNCTION__,__LINE__);
				}
				else if (waiting_) 
				{
						setAlgorithmMode(WAITING, __FUNCTION__,__LINE__);
						start_wait_time_ = ros::Time::now();
				}
				else
				{
						setAlgorithmMode(FINISHED, __FUNCTION__,__LINE__);
				}
				jog_carlin_state_ = false;
				feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 1, __FUNCTION__, __LINE__);
#if ENABLE_MINIMAL_ALGOMNGR_PRINTFS
printf ("[manageORLStatus,%d] ORL Status FINAL_STEP (2) algorithm_mode_ %d\n",__LINE__, algorithm_mode_);
#endif
		}
		else if(status == NEED_DATA)
		{
				if (jog_carlin_state_)
				{
						if (!updateJogCarlin())
						{
								ROS_ERROR("Error in JOG Cartesian");
						}
				}
				else
				{
						feedbackFn(MESSAGE_FEEDBACK::F_NEED_DATA, 0, __FUNCTION__, __LINE__);
				}
		}
    
		interpolation_data_.push_back(burned);

		if (status >= 0)
		{
				ORL_getMoveStatus (&sd_started,    /* [OUT]  If True the actual movement is started */
                           &sd_stopped,    /* [OUT]  If True the actual movement is stopped */
                           &sd_ended,      /* [OUT]  If True the actual movement is ended */
                           &sd_decPhase,   /* [OUT]  If True the actual movement is in deceleration phase */
                           &sd_flyNode,    /* [OUT]  If True a second movement is already scheduled and ready to start */
                           &sd_flyStarted, /* [OUT]  If True, if a second movement is already scheduled, indicate that the fly connection is started */
                           &nodo,          /* [OUT]  If True, if a second movement is already scheduled, indicate that the fly connection is started */
                           &nodo_fly,      /* [OUT]  If True, if a second movement is already scheduled, indicate that the fly connection is started */
                           ORL_CNTRL01, ORL_ARM1);
      	
				if ( nodo == nodo_fly_old ) // alla partenza del fly considero terminata la prima move
				{
						feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 10, __FUNCTION__, __LINE__);
				}
				nodo_fly_old = nodo_fly;
		}  
	}
	
	for(size_t i = 0; i < joints_control_.joints.size(); i++)
	{
		if (interpolation_data_.size() < 3) 
		{
			ROS_ERROR("Buffer circolare parzialmente vuoto!!!");
		}

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

		joints_control_.joints[i].velocity    = (d_angle1 + d_angle2) / DUE_PER_TS;
		joints_control_.joints[i].current     = (d_angle2 - d_angle1) / TS_SQUARE;
		joints_control_.joints[i].ff_velocity = joints_control_.joints[i].velocity;
	}
}

int AlgorithmManager::get_Strk(int strk[2][ORL_MAX_AXIS])
{
	ORL_System_Variable     orl_sys_var;

	memset(&orl_sys_var, 0x00, sizeof(ORL_System_Variable));

	for(int si_ax = 0; si_ax < ORL_MAX_AXIS; si_ax++)
	{
		sprintf(orl_sys_var.sysvar_name, "$ARM_DATA[%d].STRK_END_N[%d]", ORL_ARM1 + 1, si_ax + 1);
		orl_sys_var.ctype = ORL_INT; //  ORL_BOOL  ORL_REAL  ORL_STRING
		if(ORL_get_data(&orl_sys_var, LOCAL_VERBOSITY, ORL_CNTRL01) == ORLOPEN_RES_OK)
		{
			strk[0][si_ax] = orl_sys_var.iv;
		}
	}
	for(int si_ax = 0; si_ax < ORL_MAX_AXIS; si_ax++)
	{
		sprintf(orl_sys_var.sysvar_name, "$ARM_DATA[%d].STRK_END_P[%d]", ORL_ARM1 + 1, si_ax + 1);
		orl_sys_var.ctype = ORL_INT; //  ORL_BOOL  ORL_REAL  ORL_STRING
		if(ORL_get_data(&orl_sys_var, LOCAL_VERBOSITY, ORL_CNTRL01) == ORLOPEN_RES_OK)
		{
			strk[1][si_ax] = orl_sys_var.iv;
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
			ROS_ERROR("Error in direct kinematics");
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
	if (status < RET_OK) {
		ROS_ERROR("Error in inverse kinematics");
	}
  
	pthread_mutex_unlock(&control_mutex_);

	edo_core_msgs::JointsPositions joints_position;
	joints_position.positions.resize(joints_number_);
	for (size_t i = 0; i < joints_number_; i++) {
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
