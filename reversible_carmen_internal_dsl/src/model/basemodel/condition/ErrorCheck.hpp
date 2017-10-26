/*
 * ErrorCheck.hpp
 *
 *  Created on: Jan 9, 2015
 *      Author: josl
 */

#ifndef ERRORCHECK_HPP_
#define ERRORCHECK_HPP_

#include <../src/model/basemodel/condition/Condition.hpp>
#include <../src/model/basemodel/controller/messages/MessagesError.hpp>
#include <../src/model/basemodel/Instruction.hpp>
#include <../src/model/basemodel/State.hpp>
namespace dsl
{

class ErrorCheck : public Instruction
{
public:
	ErrorCheck(dsl::Identification, dsl::Condition * condition, dsl::MessagesError * msgError, bool isReversible, bool isSymmetric, bool isSwapped, bool isUncallLayout);
	virtual ~ErrorCheck();

	dsl::Condition * getCondition();
	void setCondition(dsl::Condition * condition);

	//Overwritten functions
	void doExecuteForward();
	void doExecuteBackward();
	std::string getType();
	void stateUpdate(dsl::State & state);


private:
	dsl::Condition * _condition;
	dsl::MessagesError * _msgError;

};

} /* namespace dsl */

#endif /* ERRORCHECK_HPP_ */
