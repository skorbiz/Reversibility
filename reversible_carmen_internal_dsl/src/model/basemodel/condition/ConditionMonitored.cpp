/*
 * ConditionMonitored.cpp
 *
 *  Created on: Jan 8, 2015
 *      Author: josl
 */

#include "ConditionMonitored.hpp"

namespace dsl {

ConditionMonitored::ConditionMonitored() :
	_hasBeenEvaluated(false),
	_isRunning(false),
	_result(false)
{
}

ConditionMonitored::~ConditionMonitored()
{
}

void ConditionMonitored::reset()
{
	_hasBeenEvaluated = false;
}

void ConditionMonitored::start()
{
	_isRunning = true;
}

void ConditionMonitored::stop()
{
	_isRunning = false;
}

// **********************************
// *** Overwritten functions
bool ConditionMonitored::getEvaluation()
{
	if(_hasBeenEvaluated == false)
	{
		_result = evaluate();
		_hasBeenEvaluated = true;
	}


	return _result;
}

// **********************************
// *** virtual functions
bool ConditionMonitored::evaluate()
{
	std::cerr << "Error, condition had not overwritten the evaluate functions" << std::endl;
	exit(-1);
	return false;
}


} /* namespace dsl */
