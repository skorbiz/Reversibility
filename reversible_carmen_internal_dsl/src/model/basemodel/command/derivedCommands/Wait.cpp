/*
 * Wait.cpp
 *
 *  Created on: Nov 6, 2014
 *      Author: josl
 */

#include "Wait.hpp"

namespace dsl {

Wait::Wait() :
	time(0.0)
{
}

Wait::Wait(double time)
{
	this->time = time;
}

Wait::~Wait()
{
}

void Wait::execute()
{
//	gen::dCommand << "executed wait" << std::endl;
	ros::Duration(time).sleep();
}

void Wait::executeBackwards()
{
//	gen::dCommand << "executed wait backwards" << std::endl;
	ros::Duration(time).sleep();
}

std::string Wait::getType()
{
	return "wait";
}


} /* namespace dsl */
