/*
 * ErrorCheckMonitoredStart.hpp
 *
 *  Created on: May 23, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_CONDITION_ERRORCHECKMONITOREDSTART_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_CONDITION_ERRORCHECKMONITOREDSTART_HPP_

#include <../src/model/basemodel/condition/ConditionMonitored.hpp>
#include <../src/model/basemodel/Instruction.hpp>
#include <../src/model/basemodel/State.hpp>


namespace dsl
{

class ErrorCheckMonitoredStart : public Instruction
{
public:
	ErrorCheckMonitoredStart(dsl::Identification, dsl::ConditionMonitored * condition, bool isReversible, bool isSymmetric, bool isSwapped, bool isUncallLayout);
	virtual ~ErrorCheckMonitoredStart();

	dsl::ConditionMonitored * getCondition();
	void setCondition(dsl::ConditionMonitored * condition);

	//Overwritten functions
	void doExecuteForward();
	void doExecuteBackward();
	std::string getType();
	void stateUpdate(dsl::State & state);

private:
	dsl::ConditionMonitored * _condition;

};

} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_CONDITION_ERRORCHECKMONITOREDSTART_HPP_ */
