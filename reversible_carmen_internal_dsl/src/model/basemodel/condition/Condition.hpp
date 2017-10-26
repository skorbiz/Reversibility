/*
 * Condition.hpp
 *
 *  Created on: Jan 8, 2015
 *      Author: josl
 */

#ifndef CONDITION_HPP_
#define CONDITION_HPP_

#include <../src/model/basemodel/State.hpp>

namespace dsl
{

class Condition
{

public:
	Condition();
	virtual ~Condition();

	virtual bool getEvaluation() = 0;
	virtual void stateUpdate(dsl::State const& state);
};

} /* namespace dsl */

#endif /* CONDITION_HPP_ */
