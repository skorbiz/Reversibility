/*
 * RobworkInterface.cpp
 *
 *  Created on: Mar 9, 2017
 *      Author: josl
 */

#include <robot_movement_interface/RobworkInterface.h>

#include <rw/math/RPY.hpp>

namespace robot_movement_interface {

RobworkInterface::RobworkInterface(std::shared_ptr<RosProxy> ros_proxy) :
		proxy(ros_proxy)
{
}

RobworkInterface::~RobworkInterface()
{
}

void RobworkInterface::movePtp(rw::math::Q q)
{
	std::string command = "joint move";
	std::string joint = toString(q);
	std::string speed = " 1 1 1 1 1 1 1 ";
	std::string blend = " 0 ";
	std::string param = joint + speed + blend;
	callCommand(command, param);
	while(is_moving());
}

void RobworkInterface::movePtp(std::vector<rw::math::Q> path)
{
	for(size_t i = 0; i < path.size(); i++)
	{
		std::string command = "joint move";
		std::string joint = toString(path[i]);
		std::string speed = " 1 1 1 1 1 1 1 ";
		std::string blend = " 0.2 ";
		std::string param = joint + speed + blend;
		callCommand(command, param);
//		while(is_at(path[i], eps) == false);
	}
	movePtp(path.back());
//	while(is_moving());
}

void RobworkInterface::moveLin(rw::math::Transform3D<> baseTtool)
{
//	std::cout << __PRETTY_FUNCTION__ <<std::endl;
//	std::cout << "Lin move from jointconf: " << getQ() << std::endl;
//	std::cout << "Lin move from baseTtool: " << getBaseTtool() << std::endl;
//	std::cout << "           to baseTtool: " << baseTtool << std::endl;
//	std::cout << std::endl;

	std::string command = "lin move";
	std::string pos = toString(baseTtool);
	std::string speed = " 1 ";
	std::string blend = " 0 ";
	std::string param = pos + speed + blend;
	callCommand(command, param);
	while(is_moving());
}

void RobworkInterface::moveLinRedundency(rw::math::Transform3D<> baseTtool, double redundency)
{
//	std::cout << __PRETTY_FUNCTION__ <<std::endl;
//	std::cout << "Lin move from jointconf: " << getQ() << std::endl;
//	std::cout << "Lin move from baseTtool: " << getBaseTtool() << std::endl;
//	std::cout << "           to baseTtool: " << baseTtool << std::endl;
//	std::cout << std::endl;

	std::string command = "linr move";
	std::string pos = toString(baseTtool);
	std::string speed = " 1 ";
	std::string blend = " 0 ";
	std::string redun = " " + std::to_string(redundency) + " ";
	std::string param = pos + speed + blend + redun;
	callCommand(command, param);
	while(is_moving());
}



void RobworkInterface::moveComplience(rw::math::Transform3D<> baseTtool)
{
	std::string command = "ptpcompliant move";
	std::string pos = toString(baseTtool);
	std::string speed = " 0.3 ";
	std::string blend = " 0 ";
	std::string comp = " 1000 5000 5000 300 300 300 ";
	std::string param = pos + speed + blend + comp;
	callCommand(command, param);
}

void RobworkInterface::stop()
{
	std::string command = "stop";
	std::string param;
	callCommand(command, param);
}

bool RobworkInterface::is_moving()
{
	std::string command = "get status";
	std::string param;
	std::string response = callCommand(command, param);

//	std::cout << "Response:" << response << std::endl;

	if(response == "moving\r\n")
		return true;
	if(response == "stopped\r\n")
		return false;

	ROS_WARN_STREAM("RobworkInterface::is_moving faild to get interpretable status. Got response: " << response );
	return false;
}



std::string RobworkInterface::callCommand(const std::string & command, const std::string & param)
{
	iiwa_driver::StringCommand::Request msg;
	msg.command = command;
	msg.parameters = param;
	iiwa_driver::StringCommand::Response res = proxy->call_iiwa_telnet(msg);
//	if(res.response != "done")
//		ROS_WARN_STREAM("faild to send move lin command. Got error code " << res.error_code << " and response '" << res.response <<"'" );
	return res.response;
}


rw::math::Q RobworkInterface::getQ()
{
	rw::math::Q q = toRw(proxy->get_msg_joint_states());
		while(q.size() != 7 && ros::ok())
		{
			//std::cout << "Waitng for reading from robot - current reading on joint is: " << q << std::endl;
			q = toRw(proxy->get_msg_joint_states());
		}
	return q;
}

rw::math::Transform3D<> RobworkInterface::getBaseTtool()
{
	rw::math::Transform3D<> baseTtool = toRw(proxy->get_msg_tool_frame());
	return baseTtool;
}

bool RobworkInterface::is_at(rw::math::Q q, double eps)
{
	return (q - getQ()).norm2() < eps;
}

bool RobworkInterface::is_at(rw::math::Transform3D<> baseTtool_target, double eps)
{
	rw::math::Transform3D<> baseTtool_current = getBaseTtool();
	if(baseTtool_current.equal(baseTtool_target, eps))
		return true;
	return false;
}

rw::math::Q RobworkInterface::toRw(sensor_msgs::JointState msg)
{
	rw::math::Q q(msg.position.size());
	for(size_t i = 0; i < msg.position.size(); i++)
		q[i] = msg.position[i];
	return q;
//	if(msg.position.size() != 7)
//		return rw::math::Q(0);
//	rw::math::Q q(7, msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6]);
//	return q;
}

rw::math::Transform3D<> RobworkInterface::toRw(robot_movement_interface::EulerFrame msg)
{
	rw::math::Vector3D<> pos(msg.x, msg.y, msg.z);
	rw::math::RPY<>rpy(msg.alpha, msg.beta, msg.gamma);
	rw::math::Transform3D<> t(pos, rpy.toRotation3D());
	return t;
}

std::string RobworkInterface::toString(rw::math::Q q)
{
	std::stringstream ss;
	ss <<q[0]<<" " <<q[1]<<" " <<q[2]<<" " <<q[3]<<" " <<q[4]<<" " <<q[5]<<" " <<q[6]<<" ";
	return ss.str();
}

std::string RobworkInterface::toString(rw::math::Transform3D<> t)
{
	rw::math::Vector3D<> pos = t.P();
	rw::math::RPY<>rpy(t.R());
	std::stringstream ss;
	ss <<pos[0]*1000<<" " <<pos[1]*1000<<" " <<pos[2]*1000<<" " <<rpy[0]<<" " <<rpy[1]<<" " <<rpy[2]<<" ";
	return ss.str();
}


} /* namespace robot_movement_interface */



