/*
 * TestInterface.cpp
 *
 *  Created on: Sep 7, 2015
 *      Author: josl
 */

#include "TestInterface.hpp"

#include <RobotGeneralInterfaceProxy.hpp>
#include <RobotCartesianInterfaceProxy.hpp>
#include <RobotServoInterfaceProxy.hpp>
#include <RobotBufferInterfaceProxy.hpp>
#include <RobotComplianceInterfaceProxy.hpp>
#include <RobotiqInterfaceProxy.hpp>
#include <GripperGeneralInterfaceProxy.hpp>
#include <DigitalIOGeneralInterfaceProxy.hpp>

//#include <caros/serial_device_si_proxy.h>
//#include <caros/proxy/ur_proxy.h>

namespace dsl {

TestInterface::TestInterface()
{
	rw::common::Ptr<ros::NodeHandle> _nodeHnd;
	_nodeHnd = rw::common::Ptr<ros::NodeHandle>(new ros::NodeHandle);
	std::string devname = "ur5";

	DigitalIOGeneralInterfaceProxy DIO(_nodeHnd, devname);
//		GripperGeneralInterfaceProxy GGI(_nodeHnd, "gripper");
//		RobotBufferInterfaceProxy RBI(_nodeHnd, devname);
//		RobotCartesianInterfaceProxy RCI(_nodeHnd, devname);
	RobotComplianceInterfaceProxy RCC(_nodeHnd, devname);
	RobotGeneralInterfaceProxy RGP(_nodeHnd, devname);
//		RobotServoInterfaceProxy RSP(_nodeHnd, devname);
	RobotiqInterfaceProxy RIQ(_nodeHnd, devname);

	ros::NodeHandle nh;
//	caros::SerialDeviceSIProxy sip(nh,"johans_devname");
//	sip.getQ();

//	caros::UrProxy urp(nh,"johans_devname2");

}

TestInterface::~TestInterface()
{
}

} /* namespace dsl */
