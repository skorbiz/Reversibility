/*
 * RosProxy.h
 *
 *  Created on: Mar 8, 2017
 *      Author: josl
 */

#ifndef D7CP_PROXYS_INCLUDE_D7CP_PROXYS_ROBOT_MOVEMENT_INTERFACE_ROSPROXY_H_
#define D7CP_PROXYS_INCLUDE_D7CP_PROXYS_ROBOT_MOVEMENT_INTERFACE_ROSPROXY_H_

#include <ros/ros.h>
#include <mutex>

#include  <iiwa_driver/StringCommand.h>

#include <robot_movement_interface/CommandList.h>
#include <robot_movement_interface/Result.h>
#include <robot_movement_interface/EulerFrame.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>

namespace robot_movement_interface
{

class RosProxy
{

public:
	RosProxy();
	virtual ~RosProxy();

	iiwa_driver::StringCommand::Response call_iiwa_telnet(const iiwa_driver::StringCommand::Request & string_command_request);

	void post_command_list(const robot_movement_interface::CommandList::ConstPtr & command_list);

	robot_movement_interface::Result get_msg_command_result();
	robot_movement_interface::EulerFrame get_msg_flange_frame();
	sensor_msgs::JointState get_msg_joint_states();
	geometry_msgs::WrenchStamped get_msg_tcp_wrench();
	robot_movement_interface::EulerFrame get_msg_tool_frame();

private:
	void callback_command_result(const robot_movement_interface::Result::ConstPtr & result);
	void callback_flange_frame(const robot_movement_interface::EulerFrame::ConstPtr & flange_frame);
	void callback_joint_states(const sensor_msgs::JointState::ConstPtr & joint_state);
	void callback_tcp_wrench(const geometry_msgs::WrenchStamped::ConstPtr & tcp_wrench);
	void callback_tool_frame(const robot_movement_interface::EulerFrame::ConstPtr & tool_frame);

private:
	ros::NodeHandle nh_;
	std::mutex mutex_;

	robot_movement_interface::Result msg_command_result_;
	robot_movement_interface::EulerFrame msg_flange_frame_;
	sensor_msgs::JointState msg_joint_states_;
	geometry_msgs::WrenchStamped msg_tcp_wrench_;
	robot_movement_interface::EulerFrame msg_tool_frame_;

	ros::ServiceClient client_iiwa_telnet_;

	ros::Publisher publisher_command_list_;

	ros::Subscriber subscriber_command_result_;
	ros::Subscriber subscriber_flange_frame_;
	ros::Subscriber subscriber_joint_states_;
	ros::Subscriber subscriber_tcp_wrench_;
	ros::Subscriber subscriber_tool_frame_;


};

} /* namespace robot_movement_interface */

#endif /* D7CP_PROXYS_INCLUDE_D7CP_PROXYS_ROBOT_MOVEMENT_INTERFACE_ROSPROXY_H_ */
