/*
 * Instruction.hpp
 *
 *  Created on: Jan 9, 2015
 *      Author: josl
 */

#ifndef INSTRUCTION_HPP_
#define INSTRUCTION_HPP_

#include <ctime>
#include <string>
#include <vector>
#include <rw/math.hpp>
#include <../src/model/basemodel/controller/messages/MessageEnvelope.hpp>
#include <../src/model/basemodel/controller/messages/MessagesContainer.hpp>
#include <../src/model/basemodel/Identification.hpp>
//#include <../src/model/basemodel/InstructionGraph.hpp>
#include <../src/model/basemodel/State.hpp>
#include <debug/Debug.hpp>
#include <color/colors.hpp>

namespace dsl
{

class Instruction
{

public:

	Instruction(dsl::Identification id, bool isReversible, bool isSymmetric, bool isSwapped, bool isUncallLayout);
	virtual ~Instruction();

	//Construction functions
	void setInstructionNext(std::shared_ptr<Instruction> ins);
	void setInstructionPrevious(std::shared_ptr<Instruction> ins);
	std::shared_ptr<Instruction> getInstructionNext() const;
	std::shared_ptr<Instruction> getInstructionPrevious() const;

	//Other functions
	dsl::Identification getIDClass();
	std::string getID();

	void initMessageContainer(dsl::MessagesContainer * container);
	dsl::State initStateForward(dsl::State state);
	dsl::State initStateReverse(dsl::State state);

	//Printing & debug functions:
	bool isUncallLayoutDEBUG() const;
	bool isSwappedLayoutDEBUG() const;
	bool isReversibleDEBUG() const;

	//Execution functions
	void executeForward();
	void executeBackward();
	bool isForwardExecutable();
	bool isBackwardExecutable();

	//Virtual funtions
	virtual std::string getType();
	virtual std::string getArgumentForward() const;
	virtual std::string getArgumentBackward() const;

protected:
	//Error functions
	void postMessageToController(dsl::MessageEnvelope * msgToController);

private:
	//Execution logic
	void executeForwardLogic();
	void executeBackwardLogic();
	virtual void doExecuteForward() = 0;
	virtual void doExecuteBackward() = 0;

	//Construction of look-ahead-features
	virtual void stateUpdate(dsl::State & state);
	virtual void stateUpdateSwapped(dsl::State & state);

	//Helper functions
	void isReversibelOrDie();
	bool isReversible();
	void toDebug(std::string executionDirection = "none");

private:
	dsl::Identification _id;
	std::shared_ptr<dsl::Instruction> _instructionNext;
	std::shared_ptr<dsl::Instruction> _instructionPrevious;

	//Reverse options
	bool _isReversible;
	bool _isSwapped;
	bool _isUncallLayout;

	//Runtime Variables
	bool _lastExecutionWasForward;


protected:
	//Controller Communications
	dsl::MessagesContainer * _messagesContainer;
	dsl::debug::Debug deb;
	dsl::debug::Debug debIB;


};



} /* namespace dsl */

//Recursive functions
//	void recursivlyClearVisited();
//	void recursivlyPassForwardState(dsl::State & state, bool isPassedForward);
//	dsl::State recursivlyPassBackwardState();


#endif /* INSTRUCTION_HPP_ */
