/*
 * getURJointconfiguration.cpp
 *
 *  Created on: Jul 4, 2016
 *      Author: josl
 */

#include "getURJointconfiguration.hpp"

#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>

getURJointconfiguration::getURJointconfiguration()
{
	rwhw::UniversalRobotsRTLogging urrt(universalrobots::CB31);

	unsigned int urrt_port = 30003;
	std::string device_ip = "192.168.1.250";

	try
	{
		urrt.connect(device_ip, urrt_port);
	}
	catch (rw::common::Exception& exp)
	{
		std::cout << "Could not connect to urrt:" << exp.what() << std::endl;
	}

	urrt.start();

	std::cout << "Connected to URRT interface" << std::endl;


	std::cout << std::setprecision(6);
	std::cout << std::fixed;

	rw::math::Q lastQ;
	while(true)
	{
		rwhw::URRTData d = urrt.getLastData();

		if(lastQ == d.qActual)
			continue;
		lastQ = d.qActual;

		std::cout << "rw::math::Q q (6, ";
		std::cout << d.qActual[0] << ", ";
		std::cout << d.qActual[1] << ", ";
		std::cout << d.qActual[2] << ", ";
		std::cout << d.qActual[3] << ", ";
		std::cout << d.qActual[4] << ", ";
		std::cout << d.qActual[5] << "); ";
		std::cout << std::endl;

//		std::cout << "rw::math::Q pose (6, ";
//		std::cout << d.toolPose[0] << ", ";
//		std::cout << d.toolPose[1] << ", ";
//		std::cout << d.toolPose[2] << ", ";
//		std::cout << d.toolPose[3] << ", ";
//		std::cout << d.toolPose[4] << ", ";
//		std::cout << d.toolPose[5] << "); ";
//		std::cout << std::endl;


//		std::cout << " driverTimeStamp: " 		<< d.driverTimeStamp;
//		std::cout << " controllerTimeStamp: " 	<< d.controllerTimeStamp;
//
//		std::cout << " |";
//		std::cout << " is_moving: " << (ur.isMoving() ? "true" : "false");
//
//		std::cout << " |";
//		std::cout << " qTarget: " 		<< d.qTarget.norm2();
//		std::cout << " dqTarget: " 		<< d.dqTarget.norm2();
//		std::cout << " ddqTarget: " 		<< d.ddqTarget.norm2();
//
//		std::cout << " |";
//		std::cout << " qActual: " 		<< d.qActual.norm2();
//		std::cout << " dqActual: " 		<< d.dqActual.norm2();
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

getURJointconfiguration::~getURJointconfiguration()
{
}

