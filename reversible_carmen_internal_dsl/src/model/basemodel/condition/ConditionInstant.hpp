/*
 * ConditionInstant.hpp
 *
 *  Created on: Jan 8, 2015
 *      Author: josl
 */

#ifndef CONDITIONINSTANT_HPP_
#define CONDITIONINSTANT_HPP_

#include <iostream>
#include <../src/model/basemodel/condition/Condition.hpp>

namespace dsl {

class ConditionInstant : public Condition
{

public:
	ConditionInstant();
	virtual ~ConditionInstant();

	//Overwritten functions
	bool getEvaluation();

protected:
	virtual bool evaluate();

private:

	bool _hasBeenEvaluated;
	bool _result;

};

} /* namespace dsl */

#endif /* CONDITIONINSTANT_HPP_ */

