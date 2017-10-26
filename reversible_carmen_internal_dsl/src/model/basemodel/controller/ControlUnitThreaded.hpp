/*
 * ControlUnitThreaded.hpp
 *
 *  Created on: Jul 4, 2016
 *      Author: josl
 */

#ifndef REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_CONTROLLER_CONTROLUNITTHREADED_HPP_
#define REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_CONTROLLER_CONTROLUNITTHREADED_HPP_

#include "ControlUnit.hpp"

namespace dsl {

class ControlUnitThreaded : public ControlUnit
{

public:
	ControlUnitThreaded(dsl::BaseProgram program);
	virtual ~ControlUnitThreaded();

	void startExecutionForward();
	void startExecutionBackward();
	void stop();

private:
	using ControlUnit::execute;
	using ControlUnit::execute_backwards;
	using ControlUnit::runProgram;
	using ControlUnit::initialise;
	using ControlUnit::step;

private:
	bool _running;
//	std::shared_ptr<ControlUnit> _controlUnit;
	boost::thread_group _tgroup;


};

} /* namespace dsl */

#endif /* REVERSIBLE_DSL_SRC_MODEL_BASEMODEL_CONTROLLER_CONTROLUNITTHREADED_HPP_ */
