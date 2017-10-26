/*
 * ErrorCheckMonitoredStart.cpp
 *
 *  Created on: May 23, 2015
 *      Author: josl
 */

#include "ErrorCheckMonitoredStart.hpp"

namespace dsl {

ErrorCheckMonitoredStart::ErrorCheckMonitoredStart(dsl::Identification id, dsl::ConditionMonitored * condition, bool isReversible, bool isSymmetric, bool isSwapped, bool isUncallLayout) :
				Instruction(id, isReversible, isSymmetric, isSwapped, isUncallLayout),
				_condition(condition)
{
}

ErrorCheckMonitoredStart::~ErrorCheckMonitoredStart()
{
}

dsl::ConditionMonitored * ErrorCheckMonitoredStart::getCondition()
{
	return _condition;
}

void ErrorCheckMonitoredStart::setCondition(dsl::ConditionMonitored * condition)
{
	_condition = condition;
}

void ErrorCheckMonitoredStart::doExecuteForward()
{
	std::cout << "Executed ErrorCheckMonitoredStart Instruction";
	_condition->start();
}

void ErrorCheckMonitoredStart::doExecuteBackward()
{
	_condition->stop();
	_condition->reset();
}

std::string ErrorCheckMonitoredStart::getType()
{
	return "error check monitored start";
}

void ErrorCheckMonitoredStart::stateUpdate(dsl::State & state)
{
}


} /* namespace dsl */
