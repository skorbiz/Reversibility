/*
 * Instruction.h
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_INSTRUCTION_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_INSTRUCTION_H_

#include <debug/Debug.hpp>
#include <model/Identification23.h>
#include <model/ExecutionResult23.h>
#include <model/InstructionExecutable.h>
#include <memory>
#include <string>

namespace model {

class Instruction
{

public:
	Instruction(Identification23 id, bool isReversible, bool isSymmetric, bool isSwapped, bool isUncallLayout);
	virtual ~Instruction();

	//Construction functions
	void setInstructionExecutable(std::shared_ptr<InstructionExecutable> executable);
	void setInstructionNext(std::shared_ptr<Instruction> ins);
	void setInstructionPrevious(std::shared_ptr<Instruction> ins);
	std::shared_ptr<Instruction> getInstructionNext() const;
	std::shared_ptr<Instruction> getInstructionPrevious() const;

	//Other functions
	Identification23 getIDClass();
	std::string getID();

	//Printing & debug functions:
	bool isUncallLayoutDEBUG() const;
	bool isSwappedLayoutDEBUG() const;
	bool isReversibleDEBUG() const;

	//Execution functions
	ExecutionResult23 executeForward();
	ExecutionResult23 executeBackward();
	bool isForwardExecutable();
	bool isBackwardExecutable();

	//Virtual funtions
	virtual std::string getType();
	virtual std::string getArgumentForward() const;
	virtual std::string getArgumentBackward() const;

private:
	//Execution logic
	ExecutionResult23 executeForwardLogic();
	ExecutionResult23 executeBackwardLogic();

	//Helper functions
	void isReversibelOrDie();
	bool isReversible();
	void toDebug(std::string executionDirection = "none");

private:
	Identification23 _id;
	std::shared_ptr<Instruction> _instructionNext;
	std::shared_ptr<Instruction> _instructionPrevious;
	std::shared_ptr<InstructionExecutable> _executable;

	//Reverse options
	bool _isReversible;
	bool _isSwapped;
	bool _isUncallLayout;

	//Runtime Variables
	bool _lastExecutionWasForward;


protected:
	//Controller Communications
	dsl::debug::Debug deb;
	dsl::debug::Debug debIB;

};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_INSTRUCTION_H_ */
