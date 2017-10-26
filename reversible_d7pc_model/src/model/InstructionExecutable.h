/*
 * InstructionExecutable.h
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_INSTRUCTIONEXECUTABLE_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_INSTRUCTIONEXECUTABLE_H_

#include <debug/Debug.hpp>
#include <memory>
#include <string>
#include <vector>
#include <model/ExecutionResult23.h>

namespace model {
class Variables;
} /* namespace model */

namespace model {

class InstructionExecutable
{

public:
	InstructionExecutable();
	virtual ~InstructionExecutable();

	void setVariableContainer(std::shared_ptr<Variables> container);
	void addArg(std::string arg);

	virtual ExecutionResult23 execute();
	virtual ExecutionResult23 executeBackwards();
	virtual std::string getArgumentForward() const;
	virtual std::string getArgumentBackward() const;
	virtual std::string getType();
	virtual void setType(std::string type);

protected:
	dsl::debug::Debug deb;
	std::string type;
	std::vector<std::string> args;
	std::shared_ptr<Variables> variables;



};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_INSTRUCTIONEXECUTABLE_H_ */
