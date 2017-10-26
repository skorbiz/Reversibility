/*
 * WaitForCondition.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: josl
 */

#include "WaitForCondition.hpp"

namespace dsl
{

WaitForCondition::WaitForCondition(ConditionInstant * condition) :
		_condition(condition)
{
}

WaitForCondition::~WaitForCondition()
{
}

void WaitForCondition::execute()
{
//	gen::dCommand << "executed waitForCondition" << std::endl;
	bool condition = true;

	while(condition)
	{
		ros::Duration(0.1).sleep();

//		if(_condition == nullptr)
//			std::cout << "nullptr"<<std::endl;

		condition = _condition->getEvaluation();
	}
}

void WaitForCondition::executeBackwards()
{
//	gen::dCommand << "Reverse executed waitForCondition" << std::endl;
	execute();
}

std::string WaitForCondition::getType()
{
	return "wait for condition";
}

} /* namespace dsl */
