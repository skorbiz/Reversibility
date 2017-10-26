/*
 * WaitForCondition.hpp
 *
 *  Created on: Jan 19, 2015
 *      Author: josl
 */

#ifndef WAITFORCONDITION_HPP_
#define WAITFORCONDITION_HPP_

#include <ros/ros.h>
#include <../src/model/basemodel/command/CommandExecutable.hpp>
#include <../src/model/basemodel/condition/ConditionInstant.hpp>

namespace dsl
{

class WaitForCondition : public CommandExecutable
{

public:
	WaitForCondition(ConditionInstant * condition);
	virtual ~WaitForCondition();

	virtual void execute();
	virtual void executeBackwards();
	virtual std::string getType();

private:
	ConditionInstant * _condition;

};

} /* namespace dsl */

#endif /* WAITFORCONDITION_HPP_ */

