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

/**this sets verbosity level of ORL library**/
#define LOCAL_VERBOSITY ORL_SILENT /**** ORL_VERBOSE or **ORL_SILENT**/

/**
 * Constructor of the manager class writes algorithm
 *  - sets the position of the configuration file (WARNING HARD CODED bad habit)
 *  - initialize ORL library
 *  - Basic handling of errors from ORL @TODO Signal a fatal error to the state
 *    machine.
 */ 
 
 bool noCartPose = true;
 edo_core_msgs::MovementCommand old_msg;
 
AlgorithmManager::AlgorithmManager(ros::NodeHandle& node):
algorithm_mode_(UNINITIALIZED)
{
	ros::NodeHandle private_nh("~");
	edo_core_msgs::JointsNumber::Request req;
	edo_core_msgs::JointsNumber::Response res;
	
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
	ros::ServiceServer get_direct_kinematics_srv = node.advertiseService("algo_direct_kinematics_srv", &AlgorithmManager::getDirectKinematics, this);
	ros::ServiceServer get_inverse_kinematics_srv = node.advertiseService("algo_inverse_kinematics_srv", &AlgorithmManager::getInverseKinematics, this);
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
	jog_carlin_state_ = false;
	move_cb_state_ = false;
	pause_state_ = false;
	interpolation_data_.resize(3);
	first_time_ = true;
	pkg_path_ = ros::package::getPath("edo_core_pkg");
	jointCalib_=0;

	private_nh.param<double>("controller_frequency", controller_frequency_, 100);
	private_nh.param<double>("state_saturation_threshold", state_saturation_threshold_, 0.5);
	private_nh.param<int>("interpolation_time_step", interpolation_time_step_, 1);

	// Duration, callback, calback-owner, oneshot, autostart
	timerCalib_ = private_nh.createTimer(ros::Duration(0.5), &AlgorithmManager::timerCallback, this, true, false);
	
	if (!initializeORL())
	{
		ROS_ERROR("Impossible to inizialize ORL");
		return;
	}

	 noCartPose = false;

	if(!getJointsNumber(req, res))
	{
		ROS_ERROR("Impossible to get joints number");
		return;
	}
	joints_number_ = res.counter;

	ROS_INFO("ORL controller initialized...");
	ros::Rate loop_rate(controller_frequency_);
	
	get_Strk(strk_);

	//test con spin asincrono
	ros::AsyncSpinner aspin(2);
	aspin.start();

	while (ros::ok())
	{
		if (algorithm_mode_ != SWITCHED_OFF){
			//update the control command
			updateControl();
			//publish the last control commands
			robot_control_publisher.publish(getCurrentControl());
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
	ROS_INFO("Terminate ORL controller...bye");
  ORL_cancel_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	ORL_terminate_controller(ORL_VERBOSE, ORL_CNTRL01);
}

bool AlgorithmManager::initializeORL()
{
	int status = ORL_initialize_controller("robot.edo", (pkg_path_ + "/config/").c_str(), LOCAL_VERBOSITY, ORL_CNTRL01); //(pkg_path_ + "/config/").c_str()
	if(status < RET_OK)
	{
		ROS_ERROR("Error %d in ORL_initialize_controller", status);
		return false;
	}
	status = ORL_set_interpolation_time(interpolation_time_step_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	if(status < RET_OK)
	{
		ROS_ERROR("Error %d in ORL_set_interpolation_time", status);
		return false;
	}
	status = ORL_select_point_accuracy(ORL_TOL_NOSETTLE, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
	if(status < RET_OK)
	{
		ROS_ERROR("Error %d in ORL_select_point_accuracy", status);
		return false;
	}
	return true;
}

bool AlgorithmManager::getJointsNumber(edo_core_msgs::JointsNumber::Request  &req, edo_core_msgs::JointsNumber::Response &res){

	ORL_System_Variable     orl_sys_var;
	sprintf(orl_sys_var.sysvar_name, "$ARM_DATA[%d].JNT_MASK", ORL_ARM1 + 1);
	orl_sys_var.ctype = ORL_INT; //  ORL_BOOL  ORL_REAL  ORL_STRING
	int status = ORL_get_data(&orl_sys_var, LOCAL_VERBOSITY, ORL_CNTRL01);
	int mask = orl_sys_var.iv;
	int jnt_num;

	if(!manageORLStatus(status))
	{
		res.counter = -1;
		return true;
	}

	for(jnt_num = 0; mask != 0; jnt_num++, mask >>= 1);
	res.counter = jnt_num; //già esegue il +1 quando esce dal ciclo for

  return true;
}

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

/**
 */
void AlgorithmManager::calibCallback(edo_core_msgs::JointCalibrationConstPtr msg)
{
	jointCalib_ |=  msg->joints_mask;
	timerCalib_.stop();
	timerCalib_.start();

}


void AlgorithmManager::timerCallback(const ros::TimerEvent& event)
{
	ROS_WARN("AZZERO ASSI");
	
	for ( int i =0; jointCalib_!=0 ; jointCalib_>>=1, i++)
	{	
		if (jointCalib_ & 1)
			hold_position_.value[i]=0.0;
	}
	
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
		feedbackFn(MESSAGE_FEEDBACK::COMMAND_RECEIVED, 0);
		move_cb_state_ = true;
		ROS_INFO("MOVE_MESSAGE_TYPE = %d command received...   sono nello stato %d", msg->movement_type, algorithm_mode_);
		//gestisco il delay, settato al controllo precedente
		//se 255 passo oltre (no_delay), altrimenti aspetto
		if (pause_state_ && msg->movement_type != MOVE_MESSAGE_TYPE::MOVE_RESUME && msg->movement_type != MOVE_MESSAGE_TYPE::MOVE_CANCEL && msg->movement_type != MOVE_MESSAGE_TYPE::MOVE_PAUSE)
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
		if(msg->movement_type != MOVE_MESSAGE_TYPE::MOVE_PAUSE && msg->movement_type != MOVE_MESSAGE_TYPE::MOVE_RESUME && msg->movement_type != MOVE_MESSAGE_TYPE::MOVE_CANCEL)
		{
			if(msg->movement_attributes.size() > 0)
			{
				delay_ = msg->movement_attributes[0];
				if (msg->movement_attributes[0] != 255 && msg->movement_attributes[0] != 0) 
				{
					waiting_ = true;
				}
			}
			else
			{
				//ROS_WARN("Delay parameter is not specified. no_delay is set as default");
				delay_ = 255;
			}
		}
//PUSH IN CODA DEI MESSAGGI ARRIVATI, in testa vengono inseriti i messaggio più prioritari (PAUSE,RESUME,CANCEL) in coda i comandi di movimento
//la coda viene svuotata dal loop che genera i target
			
		switch (msg->movement_type) 
		{
			case MOVE_MESSAGE_TYPE::MOVE_TRJNT_J:  case MOVE_MESSAGE_TYPE::MOVE_CARLIN_J:
			case MOVE_MESSAGE_TYPE::MOVE_TRJNT_C:  case MOVE_MESSAGE_TYPE::MOVE_CARLIN_C:
			case MOVE_MESSAGE_TYPE::MOVE_CARCIR_C: case MOVE_MESSAGE_TYPE::MOVE_CARCIR_J:
				msglist_.push_front(*msg);
			break;

			case MOVE_MESSAGE_TYPE::MOVE_PAUSE:
			case MOVE_MESSAGE_TYPE::MOVE_RESUME:
			case MOVE_MESSAGE_TYPE::MOVE_CANCEL:
				msglist_.push_back(*msg);
			break;

			default:
				ROS_WARN("Command not implemented, please use only accepted command type");
				feedbackFn(MESSAGE_FEEDBACK::ERROR, State::MOVETYPE_UNDEF);
				return;
			break;
		}
	
		move_cb_state_ = false;

	}
	else{
		ROS_INFO("Request for movement arrive while module was uninitialized");
		feedbackFn(MESSAGE_FEEDBACK::ERROR, State::ORL_UNINITIALIZED);
	}

}


void AlgorithmManager::jogCallback(edo_core_msgs::MovementCommandConstPtr msg){

	if(algorithm_mode_!=UNINITIALIZED)
	{
		feedbackFn(MESSAGE_FEEDBACK::COMMAND_RECEIVED, 0);
		ROS_INFO("MOVE_MESSAGE_TYPE = %d command received...", msg->movement_type);
		
		switch (msg->movement_type) 
		{
			case JOG_MESSAGE_TYPE::JOG_TRJNT: 
			case JOG_MESSAGE_TYPE::JOG_CARLIN:
				msglistJog_.push_front(*msg);
			break;

			case JOG_MESSAGE_TYPE::JOG_STOP:
				msglistJog_.push_back(*msg);
			break;

			default:
				ROS_WARN("Command not implemented, please use only accepted command type");
				feedbackFn(MESSAGE_FEEDBACK::ERROR, State::MOVETYPE_UNDEF);
				return;
			break;
		}
		
	}
	else{
		ROS_INFO("Request for movement arrive while module was uninitialized");
		feedbackFn(MESSAGE_FEEDBACK::ERROR, State::ORL_UNINITIALIZED);
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
	//ROS_WARN("dati %d, %d, %d, %d", strk[0][0], strk[1][0], strk[0][1], strk[1][1]);
	current_state_.unit_type = ORL_POSITION_LINK_DEGREE;
	for(size_t i = 0; i < msg->joints.size(); i++)
	{
		if (msg->joints[i].position <= (float)strk_[0][i]) {
			if (fabs(msg->joints[i].position - (float)strk_[0][i]) <= state_saturation_threshold_) {
				current_state_.value[i] = (float)strk_[0][i];
				ROS_WARN_THROTTLE(2, "joint %d: position saturated", i+1);
			}
			else
			{
				ROS_ERROR("Error, strk superato");
				//algorithm_mode_ = BLOCKED;//INITIALIZED;
				current_state_.value[i] = (float)strk_[0][i];
				hold_position_.value[i] = current_state_.value[i];
				//memcpy(&hold_position_, &current_state_, sizeof(ORL_joint_value));
			}
		}
		else if (msg->joints[i].position >= (float)strk_[1][i]) {
			if (fabs(msg->joints[i].position - (float)strk_[1][i]) <= state_saturation_threshold_) {
				current_state_.value[i] = (float)strk_[1][i];
				ROS_WARN_THROTTLE(2, "joint %d: position saturated", i+1);
			}
			else
			{
				ROS_ERROR("Error, strk superato");
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
	if(algorithm_mode_ == UNINITIALIZED)
		initialize(msg);

	//ad ogni aggiornamento di stato calcolo la relativa posizione cartesiana
	cartesian_pose_pub_.publish(computeCartesianPose(&current_state_));

}

/**
 * This function is called only once during the life of an instance
 * @param msg the first message received from /jnt_state
 * - sets the size of the joints vector inside the joints_control message
 *   according to the joints number in msg->size
 * - sets current control value to the value received from /jnt_state
 * - changes the state of the algorithm instance to AlgorithmManager::Mode::INITIALIZED
 */

bool AlgorithmManager::moveTrjntFn(edo_core_msgs::MovementCommand * msg)
{

	target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
	ROS_INFO("Movement type is %d", msg->movement_type);
	for(size_t i = 0; i < msg->size; i++)
	{
		target_joint_.value[i] = msg->data[i];
		ROS_INFO("Received data %d is : %f", i + 1,  target_joint_.value[i]);
	}

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

	if(first_time_)
	{
		ROS_WARN("set position");
		status = ORL_set_position(NULL, &current_state_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		first_time_ = false;
	}
	move_parameters[2] = ORL_FLY_NORMAL;

	if(!manageORLStatus(status))
	{
		return false;
	}
	else
	{
		status = setORLMovement(move_parameters, msg->movement_type, msg->ovr);
		if(!manageORLStatus(status))
		{
			return false;
		}
		/*hold_position_.unit_type = ORL_POSITION_LINK_DEGREE;
		status = ORL_get_next_interpolation_step(&hold_position_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		if(!manageORLStatus(status))
		{
			return false;
		}*/
	}
	algorithm_mode_ = MOVING;
	return true;
}


bool AlgorithmManager::moveCarlinFn(edo_core_msgs::MovementCommand * msg)
{

	memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
	target_cart_.unit_type = ORL_CART_POSITION;
	ROS_INFO("Movement type is %d", msg->movement_type);
	if (msg->size != 6) {
		ROS_ERROR("Invalid Cartesian Command, the command requires 6 data value (x, y, z, a, e, r)");
		return false;
	}
	target_cart_.x = msg->data[0];
	target_cart_.y = msg->data[1];
	target_cart_.z = msg->data[2];
	target_cart_.a = msg->data[3];
	target_cart_.e = msg->data[4];
	target_cart_.r = msg->data[5];

	if (msg->movement_attributes.size() > 0) {
		for (size_t i = 0; i < (msg->movement_attributes.size()-1); i++) {
			target_cart_.config_flags[i] = msg->movement_attributes[i+1];
		}
		target_cart_.config_flags[msg->movement_attributes.size()-1] = '\0';
	}

	ROS_INFO("Received Cartesian cmd. x: %f, y: %f, z: %f, a: %f, e: %f, r: %f", target_cart_.x, target_cart_.y, target_cart_.z, target_cart_.a, target_cart_.e, target_cart_.r);
	ROS_INFO("tags: %s", target_cart_.config_flags);

	std::vector<int> move_parameters;
	move_parameters.resize(3);

	int status = 0;
	if (delay_ == 255) {
		move_parameters[0] = ORL_FLY;
		move_parameters[1] = ORL_ADVANCE;
	}else{
		move_parameters[0] = ORL_NO_FLY;
		move_parameters[1] = ORL_WAIT;
	}

	if(first_time_)
	{
		ROS_WARN("set position");
		status = ORL_set_position(NULL, &current_state_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		first_time_ = false;
	}
	move_parameters[2] = ORL_FLY_NORMAL;

	if(!manageORLStatus(status))
	{
		return false;
	}
	else{
		status = setORLMovement(move_parameters, msg->movement_type, msg->ovr);
		if(!manageORLStatus(status))
		{
			return false;
		}

		/*
		hold_position_.unit_type = ORL_POSITION_LINK_DEGREE;
		status = ORL_get_next_interpolation_step(&hold_position_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		if(!manageORLStatus(status))
		{
			return false;
		}*/
	}
	algorithm_mode_ = MOVING;
	return true;

}


//funzione di carcir
bool AlgorithmManager::moveCarcirFn(edo_core_msgs::MovementCommand * msg){

	std::vector<int> move_parameters;
	move_parameters.resize(3);

	int status = 0;
	if (delay_ == 255) {
		move_parameters[0] = ORL_FLY;
		move_parameters[1] = ORL_ADVANCE;
	}else{
		move_parameters[0] = ORL_NO_FLY;
		move_parameters[1] = ORL_WAIT;
	}

	if(first_time_)
	{
		status = ORL_set_position(NULL, &current_state_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		first_time_ = false;
	}
	move_parameters[2] = ORL_FLY_NORMAL;

	if(!manageORLStatus(status))
	{
		return false;
	}
	else{
		//definisco i target, sia per input Cart che Jnt
		if (msg->movement_type == MOVE_CARCIR_J) {
			target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
			ROS_INFO("Movement type is %d", msg->movement_type);
			for(size_t i = 0; i < (msg->size/2); i++)
			{
				target_joint_.value[i] = msg->data[i];
				//ROS_INFO("Received data %d is : %f", i + 1,  target_joint_.value[i]);
			}
		}else if (msg->movement_type == MOVE_CARCIR_C) {
			memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
			target_cart_.unit_type = ORL_CART_POSITION;
			ROS_INFO("Movement type is %d", msg->movement_type);
			if ((msg->size/2) != 6) {
				ROS_ERROR("Invalid Cartesian Command, the command requires 6 data value (x, y, z, a, e, r)");
				return false;
			}
			target_cart_.x = msg->data[0];
			target_cart_.y = msg->data[1];
			target_cart_.z = msg->data[2];
			target_cart_.a = msg->data[3];
			target_cart_.e = msg->data[4];
			target_cart_.r = msg->data[5];

			if (msg->movement_attributes.size() > 0) {
				for (size_t i = 0; i < (msg->movement_attributes.size()-1); i++) {
					target_cart_.config_flags[i] = msg->movement_attributes[i+1];
				}
				target_cart_.config_flags[msg->movement_attributes.size()-1] = '\0';
			}


			//ROS_WARN("Received Cartesian cmd. x: %f, y: %f, z: %f, a: %f, e: %f, r: %f", target_cart_.x, target_cart_.y, target_cart_.z, target_cart_.a, target_cart_.e, target_cart_.r);
			//ROS_WARN("tags: %s", target_cart_.config_flags);
		}
		else{
			ROS_ERROR("Invalid command!");
			return false;
		}
		status = setORLMovement(move_parameters, msg->movement_type, msg->ovr);
		if(!manageORLStatus(status))
		{
			return false;
		}

		//definisco il secondo target, sia per input Cart che Jnt
		if (msg->movement_type == MOVE_CARCIR_J) {
			target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
			ROS_INFO("Movement type is %d", msg->movement_type);
			for(size_t i = (msg->size/2); i < msg->size; i++)
			{
				target_joint_.value[i] = msg->data[i];
				//ROS_INFO("Received data %d is : %f", i + 1,  target_joint_.value[i]);
			}
		}else if (msg->movement_type == MOVE_CARCIR_C) {
			memset(&target_cart_, 0, sizeof(ORL_cartesian_position));
			target_cart_.unit_type = ORL_CART_POSITION;
			ROS_INFO("Movement type is %d", msg->movement_type);
			if ((msg->size/2) != 6) {
				ROS_ERROR("Invalid Cartesian Command, the command requires 6 data value (x, y, z, a, e, r)");
				return false;
			}
			target_cart_.x = msg->data[6];
			target_cart_.y = msg->data[7];
			target_cart_.z = msg->data[8];
			target_cart_.a = msg->data[9];
			target_cart_.e = msg->data[10];
			target_cart_.r = msg->data[11];

			if (msg->movement_attributes.size() > 0) {
				for (size_t i = 0; i < (msg->movement_attributes.size()-1); i++) {
					target_cart_.config_flags[i] = msg->movement_attributes[i+1];
				}
				target_cart_.config_flags[msg->movement_attributes.size()-1] = '\0';
			}
		}
		status = setORLMovement(move_parameters, msg->movement_type, msg->ovr);
		if(!manageORLStatus(status))
		{
			return false;
		}
	}
	algorithm_mode_ = MOVING;
	return true;

}

//funzione di jog in spazio giunti
bool AlgorithmManager::jogTrjntFn(edo_core_msgs::MovementCommand *  msg)
{
    
	ORL_System_Variable Sysvar;				
	ORL_joint_value current_state_local;
	current_state_local.unit_type = ORL_POSITION_LINK_DEGREE;
	double min=1;
	
	for(size_t i = 0; i < ORL_MAX_AXIS; i++)
	{
		current_state_local.value[i]=std::max(strk_[0][i]+0.1, std::min(strk_[1][i]-0.1, current_state_.value[i]));
	}
	
	memcpy(&target_joint_, &current_state_local, sizeof(ORL_joint_value));
	target_joint_.unit_type = ORL_POSITION_LINK_DEGREE;
	ROS_INFO("Movement type is %d", msg->movement_type);
	
	for(size_t i = 0; i < msg->size; i++)
	{
		if(fabs(msg->data[i]) > 0.01)
		{	
			min=std::min(fabs(msg->data[i]),min);
			target_joint_.value[i] = (msg->data[i] < 0) ? strk_[0][i] + 0.1 : strk_[1][i] - 0.1;
		}
		ROS_WARN("Set data %d is : %f", i + 1,  target_joint_.value[i]);
	}
	
	std::vector<int> move_parameters;
	move_parameters.resize(3);

	int status = 0;
	move_parameters[0] = ORL_NO_FLY;
	move_parameters[1] = ORL_WAIT;
	move_parameters[2] = ORL_FLY_NORMAL;
	status = ORL_set_position(NULL, &current_state_local, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);

	if(!manageORLStatus(status))
	{
		return false;
	}
	else
	{
		status = setORLMovement(move_parameters, msg->movement_type, (int)(min*50));
		if(!manageORLStatus(status))
		{
			return false;
		}
	}
	
	algorithm_mode_ = MOVING;
	jog_state_ = true;

	return true;
}

bool AlgorithmManager::movePauseFn(edo_core_msgs::MovementCommand * msg)
{

	if(algorithm_mode_ != PAUSE)
	{
		int status = ORL_stop_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		if(!manageORLStatus(status))
		{
			return false;
		}
		ROS_WARN("setto pause");
		pause_state_ = true;
	}
	
	if(algorithm_mode_ != MOVING)
	{
		algorithm_mode_ = PAUSE;
	}
	
	feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 0);	
	return true;
}

bool AlgorithmManager::moveResumeFn(edo_core_msgs::MovementCommand * msg)
{
	if (algorithm_mode_ == PAUSE)
	{
		//int status = ORL_resume_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		ROS_WARN("entro");
		noCartPose = true;
		algorithm_mode_ = RECOVERY;
		ORL_cancel_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		ORL_terminate_controller(ORL_VERBOSE, ORL_CNTRL01);
		ROS_WARN("TERMINATE ORL...");
		ROS_WARN("Trying to re-initialize ORL...");
		if (!initializeORL())
		{
			ROS_ERROR("Impossible to Inizialize ORL");
		}
		noCartPose = false;
		hold_position_.unit_type = ORL_POSITION_LINK_DEGREE; //TODO aggio il problema ma chi cancella il tipo, in caso di jog cartesiano?
		int status = ORL_set_position(NULL, &hold_position_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		printf("hold_position_ : %f %f %f %f %f %f \n",hold_position_.value[0],hold_position_.value[1],hold_position_.value[2],hold_position_.value[3],hold_position_.value[4],hold_position_.value[5]);
		
		if(!manageORLStatus(status))
		{
			return false;
		}
		
		//TODO fare una funzione
		switch (old_msg.movement_type) 
		{
			case MOVE_MESSAGE_TYPE::MOVE_TRJNT_J: case MOVE_MESSAGE_TYPE::MOVE_CARLIN_J:
				if(!moveTrjntFn(&old_msg))
				{
					ROS_ERROR("Move Jnt not executed...");
					feedbackFn(MESSAGE_FEEDBACK::ERROR, State::FAILED_MOVE);
				}
			break;

			case MOVE_MESSAGE_TYPE::MOVE_TRJNT_C: case MOVE_MESSAGE_TYPE::MOVE_CARLIN_C:
				if(!moveCarlinFn(&old_msg))
				{
					ROS_ERROR("Move Lin not executed...");
					feedbackFn(MESSAGE_FEEDBACK::ERROR, State::FAILED_MOVE);
				}
			break;

			case MOVE_MESSAGE_TYPE::MOVE_CARCIR_C: case MOVE_MESSAGE_TYPE::MOVE_CARCIR_J:
				if(!moveCarcirFn(&old_msg))
				{
					ROS_ERROR("Move Cir not executed...");
					feedbackFn(MESSAGE_FEEDBACK::ERROR, State::FAILED_MOVE);
				}
			break;
		}
		pause_state_ = false;
		algorithm_mode_ = MOVING;
	}

	feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 0);
	return true;
}

bool AlgorithmManager::moveCancelFn(edo_core_msgs::MovementCommand * msg)
{
	if (algorithm_mode_ == PAUSE)
	{
		int status;
		/*
		status = ORL_cancel_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		if(!manageORLStatus(status))
		{
			return false;
		}*/
		noCartPose = true;
		algorithm_mode_ = RECOVERY;
		ORL_cancel_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		ORL_terminate_controller(ORL_VERBOSE, ORL_CNTRL01);
		ROS_WARN("TERMINATE ORL...");
		ROS_WARN("Trying to re-initialize ORL...");
		if (!initializeORL())
		{
			ROS_ERROR("Impossible to Inizialize ORL");
		}
		noCartPose = false;
		hold_position_.unit_type = ORL_POSITION_LINK_DEGREE; //TODO aggio il problema ma chi cancella il tipo, in caso di jog cartesiano?
		ORL_set_position(NULL, &hold_position_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		algorithm_mode_ = FINISHED;//finished????
		pause_state_ = false;
	}
	
	feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 0);
	return true;
}

//funzione di jog in spazio cartesiano
bool AlgorithmManager::jogCarlinFn(edo_core_msgs::MovementCommand *  msg)
{

	ORL_cartesian_position temp_cart_pos, p_new, p_step;
	float R_new[3][3];
	ORL_System_Variable Sysvar;
	double min=0;

	for(size_t i = 0; i < msg->size; i++)
	{
		min+=msg->data[i]*msg->data[i];
	}
	
	min=sqrt(min);
	
	ORL_joint_value current_state_local;
	current_state_local.unit_type = ORL_POSITION_LINK_DEGREE;
	
	for(size_t i = 0; i < ORL_MAX_AXIS; i++)
	{
		current_state_local.value[i] = std::max(strk_[0][i]+0.1, std::min(strk_[1][i]-0.1, current_state_.value[i]));
	}
	
	int status = ORL_direct_kinematics(&temp_cart_pos, &current_state_local, ORL_VERBOSE, ORL_CNTRL01, ORL_ARM1);

	p_.x = (msg->data[0]<0)?-1:((msg->data[0]>0)?+1:0);
	p_.y = (msg->data[1]<0)?-1:((msg->data[1]>0)?+1:0);
	p_.z = (msg->data[2]<0)?-1:((msg->data[2]>0)?+1:0);
	p_.a = (msg->data[3]<0)?-1:((msg->data[3]>0)?+1:0);
	p_.e = (msg->data[4]<0)?-1:((msg->data[4]>0)?+1:0);
	p_.r = (msg->data[5]<0)?-1:((msg->data[5]>0)?+1:0);

	if(!manageORLStatus(status))
	{
		return false;
	}

	temp_cart_pos.a *= M_PI / 180.0;
	temp_cart_pos.e *= M_PI / 180.0;
	temp_cart_pos.r *= M_PI / 180.0;

	vZYZm(temp_cart_pos, R_old_);

	p_step.a = MAN_ORN_STEP * p_.a * M_PI / 180.0;
	p_step.e = MAN_ORN_STEP * p_.e * M_PI / 180.0;
	p_step.r = MAN_ORN_STEP * p_.r * M_PI / 180.0;

	vXYZm(p_, R_step_);

	memset(R_new, 0x00, sizeof(float)*9);

	for(int si_i=0;si_i<3;si_i++)
		for(int si_j = 0; si_j < 3; si_j++)
		{
			for(int si_k = 0; si_k < 3; si_k++)
			{
				R_new[si_i][si_j] += R_step_[si_i][si_k] * R_old_[si_k][si_j];
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

	memcpy(&target_cart_, &temp_cart_pos, sizeof(ORL_cartesian_position));

	std::vector<int> move_parameters;
	move_parameters.resize(3);

	move_parameters[0] = ORL_FLY;
	move_parameters[1] = ORL_ADVANCE;
	move_parameters[2] = ORL_FLY_NORMAL;
	status = ORL_set_position(NULL, &current_state_local, ORL_VERBOSE, ORL_CNTRL01, ORL_ARM1);

	if(!manageORLStatus(status))
	{
		return false;
	}
	else
	{
		status = setORLMovement(move_parameters, MOVE_MESSAGE_TYPE::MOVE_CARLIN_C, std::max((int)(50*min),1));
		if(!manageORLStatus(status))
		{
			return false;
		}
	}
	algorithm_mode_ = MOVING;
	jog_carlin_state_ = true;
	jog_state_ = true;
	feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 2);
	return true;

}

//funzione di jog in spazio cartesiano
bool AlgorithmManager::jogStopFn(edo_core_msgs::MovementCommand *  msg)
{
	if (algorithm_mode_ == MOVING)
	{
		int status = ORL_stop_motion(ORL_VERBOSE, ORL_CNTRL01, ORL_ARM1);
		if(!manageORLStatus(status))
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
	algorithm_mode_ = INITIALIZED;
	ROS_INFO("First state received Algorithm is now initialized");
}


int AlgorithmManager::setORLMovement(std::vector<int> move_parameters, int movement_type, int ovr)
{

	ORL_System_Variable Sysvar;	

	ROS_INFO("set ORL movement");
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
	
	switch(movement_type)
	{
		case MOVE_MESSAGE_TYPE::MOVE_TRJNT_J: //case JOG_MESSAGE_TYPE::JOG_TRJNT:
			return ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRJNT, NULL, &target_joint_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		break;
		case MOVE_MESSAGE_TYPE::MOVE_TRJNT_C:
			return ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRJNT, &target_cart_, NULL, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		break;
		case(MOVE_MESSAGE_TYPE::MOVE_CARLIN_J):
			return ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARLIN, NULL, &target_joint_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		break;
		case(MOVE_MESSAGE_TYPE::MOVE_CARLIN_C): //case JOG_MESSAGE_TYPE::JOG_CARLIN:
			return ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARLIN, &target_cart_, NULL, ORL_VERBOSE, ORL_CNTRL01, ORL_ARM1);
		break;
		case(MOVE_MESSAGE_TYPE::MOVE_CARCIR_J):
			return ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARCIR, NULL, &target_joint_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		break;
		case(MOVE_MESSAGE_TYPE::MOVE_CARCIR_C):
			return ORL_set_move_parameters(move_parameters[0], move_parameters[1], move_parameters[2], ORL_TRCARCIR, &target_cart_, NULL, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		break;
	}
	return MOVETYPE_UNDEF; //Movement type undefined
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

bool AlgorithmManager::manageORLStatus(int const& status)
{
	if(status < RET_OK) //Error!
	{
		ROS_ERROR("ORL error %d",status);
		algorithm_mode_ = BLOCKED;
		memcpy(&hold_position_, &current_state_, sizeof(ORL_joint_value));
		feedbackFn(MESSAGE_FEEDBACK::ERROR, status);
		return false;
	}
	else if(status == FINAL_STEP)
	{
		if (pause_state_) 
		{
			algorithm_mode_ = PAUSE;
		}
		else if (waiting_) 
		{
			algorithm_mode_ = WAITING;
			start_wait_time_ = ros::Time::now();
		}
		else
		{
			algorithm_mode_ = FINISHED;
		}
		jog_carlin_state_ = false;
	}
	else if(status == NEED_DATA)
	{
		if (jog_carlin_state_)
		{
			if (!updateJogCarlin()) 
			{
				ROS_ERROR("Error in JOG Cartesian");
			}
			ROS_WARN("Update JOG Cartesian");

		}
		else
		{
			feedbackFn(MESSAGE_FEEDBACK::F_NEED_DATA, 0);
		}
	}

	return true;
}

void AlgorithmManager::feedbackFn(int type, int data)
{
	edo_core_msgs::MovementFeedback feedback;
	feedback.type = type;
	feedback.data = data;
	feedback_publisher_.publish(feedback);
}

edo_core_msgs::JointControlArray const& AlgorithmManager::getCurrentControl()
{
	return joints_control_;
}

void AlgorithmManager::updateControl()
{
	static bool debug1 = true;
	static bool debug2 = true;
	//bool ok_push;
	std_msgs::Int8 temp_algo_state;
	temp_algo_state.data = algorithm_mode_;
	algorithm_state_pub_.publish(temp_algo_state);
	edo_core_msgs::MovementCommand msg;
	
	
	//scodo i messaggi ricevuti per le move 
	while (!msglist_.empty())
	  {
	   
	    ROS_WARN("pop messaggio");
		msg=msglist_.back();
		msglist_.pop_back();			
		
		/*ok_push = false;
		
		if(old_msg.movement_type == msg.movement_type && old_msg.size == msg.size)
		{
			for(int i =0; i < msg.size; i++)
			{
				if(fabs(old_msg.data[i] - msg.data[i]) > 0.001)
				{
					ok_push = true;
					break;
				}
			}
		}
		else
		{
			ok_push = true;
		}*/

		switch (msg.movement_type) 
		{
			case MOVE_MESSAGE_TYPE::MOVE_TRJNT_J: case MOVE_MESSAGE_TYPE::MOVE_CARLIN_J:
			case MOVE_MESSAGE_TYPE::MOVE_TRJNT_C: case MOVE_MESSAGE_TYPE::MOVE_CARLIN_C:
			case MOVE_MESSAGE_TYPE::MOVE_CARCIR_C: case MOVE_MESSAGE_TYPE::MOVE_CARCIR_J:
				old_msg = msg;
			break;
		}
		/*
		if (ok_push==true)
		{*/
		   
			switch (msg.movement_type) 
			{
				case MOVE_MESSAGE_TYPE::MOVE_TRJNT_J: case MOVE_MESSAGE_TYPE::MOVE_CARLIN_J:
					if(!moveTrjntFn(&msg))
					{
						ROS_ERROR("Move Jnt not executed...");
						feedbackFn(MESSAGE_FEEDBACK::ERROR, State::FAILED_MOVE);
					}
				break;

				case MOVE_MESSAGE_TYPE::MOVE_TRJNT_C: case MOVE_MESSAGE_TYPE::MOVE_CARLIN_C:
					if(!moveCarlinFn(&msg))
					{
						ROS_ERROR("Move Lin not executed...");
						feedbackFn(MESSAGE_FEEDBACK::ERROR, State::FAILED_MOVE);
					}
				break;

				case MOVE_MESSAGE_TYPE::MOVE_CARCIR_C: case MOVE_MESSAGE_TYPE::MOVE_CARCIR_J:
					if(!moveCarcirFn(&msg))
					{
						ROS_ERROR("Move Cir not executed...");
						feedbackFn(MESSAGE_FEEDBACK::ERROR, State::FAILED_MOVE);
					}
				break;

				case MOVE_MESSAGE_TYPE::MOVE_PAUSE:
					if(!movePauseFn(&msg))
					{
						ROS_ERROR("Pause not executed...");
						feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR);
					}
				break;

				case MOVE_MESSAGE_TYPE::MOVE_RESUME:
					if(!moveResumeFn(&msg))
					{
						ROS_ERROR("Resume not executed...");
						feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR);
					}
				break;

				case MOVE_MESSAGE_TYPE::MOVE_CANCEL:
					if(!moveCancelFn(&msg))
					{
						ROS_ERROR("Cancel not executed...");
						feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR);
					}
				break;
				
				//IL: non dovrebbero esserci a questo livello messaggi non consoni... però...
				default:
					ROS_WARN("Command not implemented, please use only accepted command type");
					feedbackFn(MESSAGE_FEEDBACK::ERROR, State::MOVETYPE_UNDEF);
					return;
				break;
			}
		/*}
		else
		{
			feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 20); // ricevuto comando doppio lo scarto e mando un feedback per sbloccare state machine
		}
		*/
 
		
	}
	//scodo i messaggi arrivati per il jog
	while (!msglistJog_.empty())
	{
		msg=msglistJog_.back();
		msglistJog_.pop_back();
  
		switch (msg.movement_type) 
		{	
			case JOG_MESSAGE_TYPE::JOG_TRJNT:
				if(!jogTrjntFn(&msg))
				{
					ROS_ERROR("Move JNTtrj not executed...");
					feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR);
				}
			break;

			case JOG_MESSAGE_TYPE::JOG_CARLIN:
				if(!jogCarlinFn(&msg))
				{
					ROS_ERROR("Move CARTtrj not executed...");
					feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR);
				}
			break;

			case JOG_MESSAGE_TYPE::JOG_STOP:
				if(!jogStopFn(&msg))
				{
					ROS_ERROR("Move JOG stop not executed...");
					feedbackFn(MESSAGE_FEEDBACK::ERROR, State::GENERIC_ERROR);
				}
			break;
			
			//IL: non dovrebbero esserci a questo livello messaggi non consoni... però...
			default:
				ROS_WARN("Command not implemented, please use only accepted command type");
				feedbackFn(MESSAGE_FEEDBACK::ERROR, State::MOVETYPE_UNDEF);
				return;
			break;
		}
	}
	
	if(algorithm_mode_ == MOVING)
	{
		if(debug1)
		{
			ROS_INFO("ALGO MODULE STARTED SENDING ORL CONTROL");
			debug1 = false;
			debug2 = true;
		}
		setControl();
	}
	else if(algorithm_mode_ == INITIALIZED)
	{
		if(debug2)
		{
			ROS_INFO("ALGO MODULE STARTED SENDING KEEP POSITION MESSAGES");
			debug2 = false;
			debug1 = true;
		}
		keepPosition();
	}
	else if (algorithm_mode_ == WAITING) 
	{
		keepPosition();
		ros::Duration d = ros::Time::now() - start_wait_time_;
		if(d.toSec() >= (double)delay_){
			waiting_ = false;
			algorithm_mode_ = FINISHED;
			delay_ = 255;
		}
	}
	else if (algorithm_mode_ == BLOCKED) 
	{
		keepPosition();
		waiting_ = false;
		algorithm_mode_ = INITIALIZED;
	}
	else if (algorithm_mode_ == PAUSE) 
	{
		keepPosition();
	}

	else if (algorithm_mode_ == FINISHED) 
	{
		if (jog_state_)
		{
			noCartPose = true;
			algorithm_mode_ = RECOVERY;
            ORL_cancel_motion(LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
			ORL_terminate_controller(ORL_VERBOSE, ORL_CNTRL01);
			ROS_WARN("TERMINATE ORL...");
			ROS_WARN("Trying to re-initialize ORL...");
			if (!initializeORL())
			{
				ROS_ERROR("Impossible to Inizialize ORL");
			}
			noCartPose = false;
			hold_position_.unit_type = ORL_POSITION_LINK_DEGREE; //TODO aggio il problema ma chi cancella il tipo, in caso di jog cartesiano?
			ORL_set_position(NULL, &hold_position_, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
			jog_state_ = false;
		}

		keepPosition();
		algorithm_mode_ = INITIALIZED;
	}
}

bool flag_end = false;

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
		 nodo_fly_old;   
	
	for(size_t i = 0; i < 10 && status != FINAL_STEP; i++)
	{
		status = ORL_get_next_interpolation_step(&burned, LOCAL_VERBOSITY, ORL_CNTRL01, ORL_ARM1);
		
		if(!manageORLStatus(status))
		{
			ROS_ERROR("Error BRUTTO");
			return;
		}
		
		ORL_getMoveStatus (&  sd_started,    /* [OUT]  If True the actual movement is started */
                           &  sd_stopped,    /* [OUT]  If True the actual movement is stopped */
                           &  sd_ended,      /* [OUT]  If True the actual movement is ended */
                           &  sd_decPhase,   /* [OUT]  If True the actual movement is in deceleration phase */
                           &  sd_flyNode,    /* [OUT]  If True a second movement is already scheduled and ready to start */
                           &  sd_flyStarted, /* [OUT]  If True, if a second movement is already scheduled, indicate that the fly connection is started */
                           &  nodo,   		 /* [OUT]  If True, if a second movement is already scheduled, indicate that the fly connection is started */
                           &  nodo_fly, 	 /* [OUT]  If True, if a second movement is already scheduled, indicate that the fly connection is started */
                              ORL_CNTRL01, ORL_ARM1);
		
		interpolation_data_.push_back(burned);
			
		if ( nodo == nodo_fly_old ) // alla partenza del fly considero terminata la prima move
		{
		   feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 10);
		}
		
		nodo_fly_old = nodo_fly;
	}
	
	if (status == FINAL_STEP )
	{  
		feedbackFn(MESSAGE_FEEDBACK::COMMAND_EXECUTED, 1);
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

		joints_control_.joints[i].velocity    = (d_angle1 + d_angle2) / 0.002;
		joints_control_.joints[i].current     = (d_angle2 - d_angle1) / 0.000001;
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
		if(ORL_get_data(&orl_sys_var, ORL_SILENT, ORL_CNTRL01) == ORLOPEN_RES_OK)
		{
			strk[0][si_ax] = orl_sys_var.iv;
		}
	}
	for(int si_ax = 0; si_ax < ORL_MAX_AXIS; si_ax++)
	{
		sprintf(orl_sys_var.sysvar_name, "$ARM_DATA[%d].STRK_END_P[%d]", ORL_ARM1 + 1, si_ax + 1);
		orl_sys_var.ctype = ORL_INT; //  ORL_BOOL  ORL_REAL  ORL_STRING
		if(ORL_get_data(&orl_sys_var, ORL_SILENT, ORL_CNTRL01) == ORLOPEN_RES_OK)
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
	cartesian_pose.x = 0;
	cartesian_pose.y = 0;
	cartesian_pose.z = 0;
	cartesian_pose.a = 0;
	cartesian_pose.e = 0;
	cartesian_pose.r = 0;
		
    if (noCartPose == false)
	{
		ORL_cartesian_position cartesian_state;
		cartesian_state.unit_type = ORL_CART_POSITION;
		int status = ORL_direct_kinematics(&cartesian_state, joint_state ,ORL_SILENT, ORL_CNTRL01, ORL_ARM1);//LOCAL_VERBOSITY
		if (status < RET_OK) 
		{
			ROS_ERROR("Error in direct kinematics");
		}
		
		cartesian_pose.x = cartesian_state.x;
		cartesian_pose.y = cartesian_state.y;
		cartesian_pose.z = cartesian_state.z;
		cartesian_pose.a = cartesian_state.a;
		cartesian_pose.e = cartesian_state.e;
		cartesian_pose.r = cartesian_state.r;
		cartesian_pose.config_flags = cartesian_state.config_flags;
		
	}
	
	return cartesian_pose;
}

edo_core_msgs::JointsPositions AlgorithmManager::computeJointValue(edo_core_msgs::CartesianPose cartesian_pose)
{
	ORL_joint_value joint_state;
	ORL_cartesian_position cartesian_state;
	
	cartesian_state.unit_type = ORL_CART_POSITION;
	cartesian_state.x = cartesian_pose.x;
	cartesian_state.y = cartesian_pose.y;
	cartesian_state.z = cartesian_pose.z;
	cartesian_state.a = cartesian_pose.a;
	cartesian_state.e = cartesian_pose.e;
	cartesian_state.r = cartesian_pose.r;
	memcpy(cartesian_state.config_flags, cartesian_pose.config_flags.c_str(), 80);

	joint_state.unit_type = ORL_POSITION_LINK_DEGREE;
	
	int status = ORL_inverse_kinematics(&cartesian_state, &joint_state, ORL_SILENT, ORL_CNTRL01, ORL_ARM1);//LOCAL_VERBOSITY
	if (status < RET_OK) {
		ROS_ERROR("Error in inverse kinematics");
	}

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
	int temp_status = setORLMovement(move_parameters, MOVE_MESSAGE_TYPE::MOVE_CARLIN_C,0);


	if(!manageORLStatus(temp_status))
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
		algorithm_mode_ = SWITCHED_OFF; // Stop control
	} else if(req.mode == 2 && algorithm_mode_ == SWITCHED_OFF) {// Client requests to release the control
		res.result = 0;
		algorithm_mode_ = UNINITIALIZED; // Ready to initialize ORL
	} else {
		res.result = 1;
	}
	return true;
}
