/*
 * Assignment.h
 *
 *  Created on: Jun 20, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_ASSIGNMENT_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_ASSIGNMENT_H_

#include <string>
#include "../InstructionExecutable.h"

namespace model {

class Assignment : public InstructionExecutable
{
public:
	Assignment(std::string text, std::string variable);
	virtual ~Assignment();
	ExecutionResult23 execute();
	ExecutionResult23 executeBackwards();

private:
	std::string _textToAssign;
	std::string _variable;


};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_ASSIGNMENT_H_ */
