/*
 * ConditionMonitored.hpp
 *
 *  Created on: Jan 8, 2015
 *      Author: josl
 */

#ifndef CONDITIONMONITORED_HPP_
#define CONDITIONMONITORED_HPP_

#include <../src/model/basemodel/condition/Condition.hpp>

namespace dsl
{

class ConditionMonitored : public Condition
{

public:
	ConditionMonitored();
	virtual ~ConditionMonitored();

	void reset();
	void start();
	void stop();
	bool getEvaluation();

private:
	virtual bool evaluate();

	bool _hasBeenEvaluated;
	bool _isRunning;
	bool _result;

};

} /* namespace dsl */

#endif /* CONDITIONMONITORED_HPP_ */
