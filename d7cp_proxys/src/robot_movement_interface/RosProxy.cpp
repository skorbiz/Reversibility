/*
 * RosProxy.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: josl
 */

#include <robot_movement_interface/RosProxy.h>

namespace robot_movement_interface {

RosProxy::RosProxy()
{
	nh_ = ros::NodeHandle();

	client_iiwa_telnet_ = nh_.serviceClient<iiwa_driver::StringCommand>("iiwa_telnet");

	publisher_command_list_  = nh_.advertise<robot_movement_interface::CommandList>("command_list", 100);

	subscriber_command_result_  = nh_.subscribe("command_result", 10, &RosProxy::callback_command_result, this);
	subscriber_flange_frame_ = nh_.subscribe("flange_frame", 1, &RosProxy::callback_flange_frame, this);
	subscriber_joint_states_ = nh_.subscribe("joint_states", 1, &RosProxy::callback_joint_states, this);
	subscriber_tcp_wrench_ = nh_.subscribe("tcp_wrench", 1, &RosProxy::callback_tcp_wrench, this);
	subscriber_tool_frame_ = nh_.subscribe("tool_frame", 1, &RosProxy::callback_tool_frame, this);

	if(!client_iiwa_telnet_.waitForExistence(ros::Duration(5)))
		ROS_WARN("RosProxy timed out in wating for service client iiwa_telnet ");

}

RosProxy::~RosProxy()
{
}

iiwa_driver::StringCommand::Response RosProxy::call_iiwa_telnet(const iiwa_driver::StringCommand::Request & string_command_request)
{
	iiwa_driver::StringCommand::Response response;
	client_iiwa_telnet_.call(string_command_request, response);
	return response;
}

void RosProxy::post_command_list(const robot_movement_interface::CommandList::ConstPtr & command_list)
{
	publisher_command_list_.publish(command_list);
}

///////////////////////////////////////////////////////////////////////////
//GET topic outputs

robot_movement_interface::Result RosProxy::get_msg_command_result()
{
	std::lock_guard<std::mutex> lock(mutex_);
	return msg_command_result_;
}

robot_movement_interface::EulerFrame RosProxy::get_msg_flange_frame()
{
	std::lock_guard<std::mutex> lock(mutex_);
	return msg_flange_frame_;
}

sensor_msgs::JointState RosProxy::get_msg_joint_states()
{
	std::lock_guard<std::mutex> lock(mutex_);
	return msg_joint_states_;
}

geometry_msgs::WrenchStamped RosProxy::get_msg_tcp_wrench()
{
	std::lock_guard<std::mutex> lock(mutex_);
	return msg_tcp_wrench_;
}

robot_movement_interface::EulerFrame RosProxy::get_msg_tool_frame()
{
	std::lock_guard<std::mutex> lock(mutex_);
	return msg_tool_frame_;
}

///////////////////////////////////////////////////////////////////////////
//CALLBACK for topics

void RosProxy::callback_command_result(const robot_movement_interface::Result::ConstPtr & command_result)
{
	std::lock_guard<std::mutex> lock(mutex_);
	msg_command_result_ = *command_result;
}

void RosProxy::callback_flange_frame(const robot_movement_interface::EulerFrame::ConstPtr & flange_frame)
{
	std::lock_guard<std::mutex> lock(mutex_);
	msg_flange_frame_ = *flange_frame;
}

void RosProxy::callback_joint_states(const sensor_msgs::JointState::ConstPtr & joint_state)
{
	std::lock_guard<std::mutex> lock(mutex_);
	msg_joint_states_ = *joint_state;
}

void RosProxy::callback_tcp_wrench(const geometry_msgs::WrenchStamped::ConstPtr & tcp_wrench)
{
	std::lock_guard<std::mutex> lock(mutex_);
	msg_tcp_wrench_ = *tcp_wrench;
}

void RosProxy::callback_tool_frame(const robot_movement_interface::EulerFrame::ConstPtr & tool_frame)
{
	std::lock_guard<std::mutex> lock(mutex_);
	msg_tool_frame_ = *tool_frame;
}


} /* namespace robot_movement_interface */
