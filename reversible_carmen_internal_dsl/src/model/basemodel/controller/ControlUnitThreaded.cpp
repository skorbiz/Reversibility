/*
 * ControlUnitThreaded.cpp
 *
 *  Created on: Jul 4, 2016
 *      Author: josl
 */

#include "ControlUnitThreaded.hpp"

namespace dsl {

ControlUnitThreaded::ControlUnitThreaded(dsl::BaseProgram program)
	: ControlUnit(program)
	, _running(true)
{
}

ControlUnitThreaded::~ControlUnitThreaded()
{
	stop();
}

void ControlUnitThreaded::startExecutionForward()
{
	_tgroup.create_thread( boost::bind( &ControlUnitThreaded::execute, this) );
	_running = true;
}

void ControlUnitThreaded::startExecutionBackward()
{
	_tgroup.create_thread( boost::bind( &ControlUnitThreaded::execute_backwards, this) );
	_running = true;
}

void ControlUnitThreaded::stop()
{
	_running = false;
	_tgroup.join_all();
}




} /* namespace dsl */
