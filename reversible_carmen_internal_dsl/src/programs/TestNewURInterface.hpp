/*
 * TestNewURInterface.hpp
 *
 *  Created on: Apr 4, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_PROGRAMS_TESTNEWURINTERFACE_HPP_
#define REVERSIBLE_DSL_SRC_PROGRAMS_TESTNEWURINTERFACE_HPP_

#include <rwhw/universalrobots/UniversalRobotsRTLogging.hpp>
#include <rwhw/universalrobots/URCallBackInterface.hpp>


class TestNewURInterface {
public:
	TestNewURInterface();
	virtual ~TestNewURInterface();
	void printURRTData();
	void printURData();

private:
	rwhw::UniversalRobotsRTLogging urrt;
	rwhw::URCallBackInterface ur;
};

#endif /* REVERSIBLE_DSL_SRC_PROGRAMS_TESTNEWURINTERFACE_HPP_ */
