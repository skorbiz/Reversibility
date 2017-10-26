/*
 * RosAutoSpinner.cpp
 *
 *  Created on: Apr 5, 2017
 *      Author: josl
 */

#include "RosAutoSpinner.h"

RosAutoSpinner::RosAutoSpinner(int argc, char **argv, std::string nodeName)
{
	ros::init(argc, argv, nodeName);
	ros::start();
	_runSpinThread = true;
	_spinThread = boost::thread(&RosAutoSpinner::threadFunc, this);
}

RosAutoSpinner::~RosAutoSpinner()
{
	_runSpinThread = false;
	_spinThread.join();
}

void RosAutoSpinner::threadFunc()
{
	while (_runSpinThread)
	{
		ros::spinOnce();
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
	}
}
