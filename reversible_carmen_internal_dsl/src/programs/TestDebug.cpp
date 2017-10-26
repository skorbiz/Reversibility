/*
 * TestDebug.cpp
 *
 *  Created on: Sep 7, 2015
 *      Author: josl
 */

#include "TestDebug.hpp"

namespace dsl {

TestDebug::TestDebug()
{
	dsl::debug::Debug deba;
	deba << "abc" << std::endl;

	dsl::debug::Debug deb;
	deb.setColor(dsl::color::RED);
	deb.setPrefix("[main]");
	deb << 123 << std::endl;
	deb << 456 << std::endl;
	std::cout << "ccc" << std::endl;
	deb.setColorText(dsl::color::BLUE);
	deb << 789 << std::endl;
	deb.disable();
	deb << 101112 << std::endl;
	deb.enable();
	deb << 131415 << std::endl;
}

TestDebug::~TestDebug()
{
	// TODO Auto-generated destructor stub
}

} /* namespace dsl */
