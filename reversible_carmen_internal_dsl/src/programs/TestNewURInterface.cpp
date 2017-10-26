/*
 * TestNewURInterface.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: josl
 */

#include "TestNewURInterface.hpp"

#include <ros/duration.h>
#include <rw/common/Exception.hpp>
#include <rw/math/Q.hpp>
#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>
#include <rwhw/universalrobots/URDefinitions.hpp>
#include <iostream>
#include <string>
#include <boost/thread.hpp>


void TestNewURInterface::printURRTData()
{
	std::cout << std::setprecision(6);
	std::cout << std::fixed;

	while(true)
	{
		rwhw::URRTData d = urrt.getLastData();
		std::cout << " driverTimeStamp: " 		<< d.driverTimeStamp;
		std::cout << " controllerTimeStamp: " 	<< d.controllerTimeStamp;

		std::cout << " |";
		std::cout << " is_moving: " << (ur.isMoving() ? "true" : "false");

		std::cout << " |";
		std::cout << " qTarget: " 		<< d.qTarget.norm2();
		std::cout << " dqTarget: " 		<< d.dqTarget.norm2();
		std::cout << " ddqTarget: " 		<< d.ddqTarget.norm2();

		std::cout << " |";
		std::cout << " qActual: " 		<< d.qActual.norm2();
		std::cout << " dqActual: " 		<< d.dqActual.norm2();
//
//		std::cout << " |";
//		std::cout << " torqueTarget: " 		<< d.torqueTarget.norm2();
//
//		std::cout << " |";
//		std::cout << " accValues: " 	<< d.accValues.norm2();
//		std::cout << " tcpForce: " 		<< d.tcpForce.norm2();
//		std::cout << " tcpSpeed: " 		<< d.tcpSpeed.norm2();
//		std::cout << " tcpSpeedActual: " << d.tcpSpeedActual.norm2();
//		std::cout << " toolPose: " 		<< d.toolPose.norm2();
//		std::cout << " toolPoseActual: " << d.toolPoseActual.norm2();
//
//		std::cout << " |";
//		std::cout << " motorTemperatures: " 		<< d.motorTemperatures;
//
//		std::cout << " |";
//		std::cout << " robotMode: " 			<< d.robotMode;
//		std::cout << " jointModes: " 		<< d.jointModes;
//		std::cout << " safetyMode: " 		<< d.safetyMode;
//
//		std::cout << " |";
//		std::cout << " speedScaling: " 		<< d.speedScaling;
//		std::cout << " linearMomentumNorm: " << d.linearMomentumNorm;
//
//		std::cout << " |";
//		std::cout << " iTarget: " 		<< d.iTarget.norm2();
//		std::cout << " iActual: " 		<< d.iActual.norm2();
//		std::cout << " iControl: " 		<< d.iControl.norm2();
//		std::cout << " iRobot: " 		<< d.iRobot;
//		std::cout << " vMain: " 		<< d.vMain;
//		std::cout << " vRobot: " 		<< d.vRobot;
//		std::cout << " vActual: " 		<< d.vActual.norm2();
//
//		std::cout << " |";
//		std::cout << "digIn: " 		<< d.digIn;
		std::cout << std::endl;
	}
}

void TestNewURInterface::printURData()
{
	std::cout << std::setprecision(6);
	std::cout << std::fixed;

	rwhw::URPrimaryInterface & urp = ur.getPrimaryInterface();

	while(!urp.hasData())
		ros::Duration(1).sleep();

	while(true)
	{
		rwhw::UniversalRobotsData d = urp.getLastData();
		std::cout << " robotMode: " << d.robotMode;
		std::cout << " controlMode: " << d.controlMode;

		for(int i = 0; i < 1; i++)
		{
			std::cout << "joint: " << i;
			std::cout << " mode: " << d.jointMode[i];
			std::cout << " current: " << d.jointCurrent[i];
			std::cout << " voltage: " << d.jointVoltage[i];
			std::cout << " motorT: " << d.jointMotorTemperature[i];
			std::cout << " microT: " << d.jointMicroTemperature[i];
		}

		std::cout << " speedScaling: " << d.speedScaling;
		std::cout << " targetSpeedFraction: " << d.targetSpeedFraction;


		std::cout << " physical: " << (d.physical ? "true" : "false");
		std::cout << " robotPowerOn: " << (d.robotPowerOn ? "true" : "false");
		std::cout << " emergencyStopped: " << (d.emergencyStopped ? "true" : "false");
		std::cout << " securityStopped: " << (d.securityStopped ? "true" : "false");
		std::cout << " protectiveStopped: " << (d.protectiveStopped ? "true" : "false");
		std::cout << " programRunning: " << (d.programRunning ? "true" : "false");
		std::cout << " programPaused: " << (d.programPaused ? "true" : "false");
		std::cout << std::endl;
	}
}

TestNewURInterface::TestNewURInterface() :
		urrt(universalrobots::CB31),
		ur(universalrobots::CB31)
{

	std::cout << "Starting test ur" << std::endl;

	unsigned int urrt_port = 30003;
	unsigned int ur_port = 30001;
	std::string device_ip = "192.168.1.250";
	std::string callback_ip = "192.168.1.240";
	unsigned int numeric_callback_port = 33333;


	try
	{
		urrt.connect(device_ip, urrt_port);
	}
	catch (rw::common::Exception& exp)
	{
		std::cout << "Could not connect to urrt:" << exp.what() << std::endl;
	}

	try
	{
		ur.connect(device_ip, ur_port);
	}
	catch (rw::common::Exception& exp)
	{
		std::cout << "Could not connect to urrt:" << exp.what() << std::endl;
	}

	ur.startCommunication(callback_ip, numeric_callback_port);
	urrt.start();

	std::cout << "Connected to UR interface" << std::endl;

	//Original test set
	rw::math::Q q1 (6, 2.093392, -0.972999, 1.158061, 1.277256, 1.533655, -0.950996);
	rw::math::Q q2 (6, 1.429307, -1.292614, 1.641497, 1.163161, 1.475594, -1.615585);
	rw::math::Q q3 (6, 1.429307, -1.292614, 1.641497, 1.163161, 1.475594, -1.615585);
	rw::math::Q q4 (6, 2.231821, -1.233570, 1.208577, 0.881294, 1.654781, 0.526713);
//	rw::math::Q q4(6, 1.40169, -1.2605, 0.56003, -0.7888, 1.57509, 2.60431);

	rw::common::Ptr<boost::thread> _thread;
	_thread = rw::common::ownedPtr(new boost::thread(&TestNewURInterface::printURRTData, this));
//	_thread = rw::common::ownedPtr(new boost::thread(&TestNewURInterface::printURData, this));

	std::cout << "Spawend print thread" << std::endl;

			ur.moveL(q1,0.12,1.1337);
			ros::Duration(5).sleep();

			ur.moveL(q2,0.12,1.1337);
			ros::Duration(5).sleep();

			ur.moveL(q3,0.12,1.1337);
			ros::Duration(5).sleep();

			ur.moveL(q4,0.12,1.1337);
			ros::Duration(5).sleep();


			ros::Duration(2).sleep();
			std::cout << "step 2" << std::endl;

			ur.moveJ(q1,0.12,1.1337);
			ros::Duration(10).sleep();

			ur.moveJ(q2,0.12,1.1337);
			ros::Duration(10).sleep();

			ur.moveJ(q3,0.12,1.1337);
			ros::Duration(10).sleep();

			ur.moveJ(q4,0.12,1.1337);
			ros::Duration(10).sleep();


//		}
//		if(i%3 == 2)
			ros::Duration(2).sleep();

		ros::Duration(4).sleep();
		std::cout << "step 3" << std::endl;
//	}



//	std::cout << "Finished test ur" << std::endl;

}

TestNewURInterface::~TestNewURInterface()
{
}

