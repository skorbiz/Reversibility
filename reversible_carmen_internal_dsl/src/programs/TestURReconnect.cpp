/*
 * TestURReconnect.cpp
 *
 *  Created on: Jun 24, 2016
 *      Author: josl
 */

#include "TestURReconnect.hpp"

#include <rw/RobWork.hpp>
#include <ros/ros.h>

#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/URDefinitions.hpp>


TestURReconnect::TestURReconnect()
{

	unsigned int urrt_port = 30003;
	unsigned int ur_port = 30001;
	std::string device_ip = "192.168.1.250";
	std::string callback_ip = "192.168.1.240";
	unsigned int numeric_callback_port = 33333;

	std::shared_ptr<rwhw::URCallBackInterface> ur = std::make_shared<rwhw::URCallBackInterface>(universalrobots::CB31);

	std::cout << "Connecting to UR interface" << std::endl;
	try
	{
		ur->connect(device_ip, ur_port);
	}
	catch (rw::common::Exception& exp)
	{
		std::cout << "Could not connect to ur:" << exp.what() << std::endl;
	}

	ur->startCommunication(callback_ip, numeric_callback_port);
	std::cout << "Started commonication on UR interface" << std::endl;


	rw::math::Q q2 	(6, 2.61505198, -1.2303603, 1.69783592, -2.0270512, -1.5772302, 2.60829782 );
	ur->moveJ(q2);

	ros::Duration(1).sleep();
	std::cout << std::endl;
	std::cout << "##  reconnect to UR interface ##" << std::endl;


	//ur->~URCallBackInterface();
//	ur = std::make_shared<rwhw::URCallBackInterface>(universalrobots::CB31);
	ur->stopCommunication();

	ros::Duration(1).sleep();

	std::cout << "1) Stoped connection" << std::endl;

	try
	{
		ur->connect(device_ip, ur_port);
	}
	catch (rw::common::Exception& exp)
	{
		std::cout << "Could not connect to urrt:" << exp.what() << std::endl;
	}

	std::cout << "2) connected" << std::endl;


	ur->startCommunication(callback_ip, numeric_callback_port);
	std::cout << "3) ReConnected to UR interface" << std::endl;

	ros::Duration(5).sleep();

	rw::math::Q q (6, 2.365823, -1.64587, 1.880681, -1.78785, -1.55238, 2.382796 );

	ur->moveJ(q);

	ros::Duration(5).sleep();

}

TestURReconnect::~TestURReconnect()
{
}

