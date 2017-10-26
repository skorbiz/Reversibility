/*
 * Move.cpp
 *
 *  Created on: Nov 6, 2014
 *      Author: josl
 */

#include "Move.hpp"
#include <ros/ros.h>

namespace dsl
{


Move::Move(std::shared_ptr<rwhw::UniversalRobotsRTLogging> urrt, std::shared_ptr<rwhw::URCallBackInterface> ur, rw::math::Q qTo) :
		urrt(urrt),
		ur(ur),
		qTo(qTo),
//		speed(15.00),	acceleration(1.5)		// Very fast for demos (100 test)
//		speed(15.00),	acceleration(2.5)		// Very fast (100 test)
		speed( 2.00),	acceleration(0.5)		// fast		 (for tested programs)
//		speed( 0.10),	acceleration(0.1)
//		speed( 0.05),	acceleration(0.1)		//Slow
//		speed( 0.01),	acceleration(0.1)		//Painfully slow
{
}

Move::~Move()
{
}

void Move::waitForMovement(rw::math::Q q)
{
	while(urrt->hasData() == false)
	{
		std::cout << "." << urrt->getLastData().qTarget << std::endl;
		ros::Duration(0.5).sleep();
	}

	while(true)
	{
		rw::math::Q qC = urrt->getLastData().qActual;
		double norm = (q-qC).norm2();
		std::cout << norm << std::endl;
		ros::Duration(0.25).sleep();
		if(norm < 0.001)
			break;
	}
}

bool is_first_move = true;

void Move::execute()
{
	if(qTo.size() == 0)
	{
		std::cerr << "Can't execute base command 'move' with a jointConfiguration of length 0" << std::endl;
		exit(-1);
	}

	if(is_first_move)
		ur->moveJ(qTo,speed,acceleration);
	is_first_move = false;

	ur->moveL(qTo,speed,acceleration);
	waitForMovement(qTo);

	deb << "Move start: " << qFrom << std::endl;
	deb << "Move end:   " << qTo << std::endl;
}


void Move::executeBackwards()
{
	deb << "Move start: " << qTo << std::endl;
	deb << "Move end:   " << qFrom << std::endl;

	if(qFrom.size() == 0)
	{
		std::cerr << "Warning: move executed qTo instead of qFrom." << std::endl;
		ur->moveL(qTo,speed,acceleration);
		waitForMovement(qTo);
		return;
	}

	ur->moveL(qTo,speed,acceleration);
	ur->moveL(qFrom,speed,acceleration);
	waitForMovement(qFrom);

}

void Move::stateUpdate(dsl::State & state)
{
	qFrom = state.getQ();
	state.update(qTo);
}

void Move::stateUpdateSwapped(dsl::State & state)
{
	state.update(qFrom);
}

std::string Move::getType()
{
	return "move";
}

std::string Move::getArgumentForward() const
{
	std::stringstream s;
	s << std::fixed;
	s << std::setprecision(2);
	s << qTo;
	return s.str();
}

std::string Move::getArgumentBackward() const
{
	std::stringstream s;
	s << std::fixed;
	s << std::setprecision(2);
	s << qFrom;
	return s.str();
}



} /* namespace dsl */

