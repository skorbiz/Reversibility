/*
 * ControlUnit.hpp
 *
 *  Created on: Dec 1, 2014
 *      Author: josl
 */

#ifndef CONTROLUNIT_HPP_
#define CONTROLUNIT_HPP_

#include <iostream>
#include <vector>
#include <random>
#include <boost/thread.hpp>

#include <color/colors.hpp>
#include <debug/Debug.hpp>
#include <../src/model/basemodel/BaseProgram.hpp>
#include <../src/model/basemodel/controller/errorHandles/ErrorHandleDeterministic.hpp>
#include <../src/model/basemodel/controller/errorHandles/ErrorHandleRandom.hpp>
#include <../src/model/basemodel/controller/errorHandles/ManualErrorInjecter.hpp>
#include <../src/model/basemodel/controller/messages/MessagesContainer.hpp>
#include <../src/model/basemodel/controller/messages/MessageEnvelope.hpp>
#include <../src/model/basemodel/controller/messages/MessagesError.hpp>
#include <../src/model/basemodel/Instruction.hpp>
#include <../src/model/basemodel/InstructionUtilities.hpp>

namespace dsl
{

class ControlUnit
{
	enum class Direction
	{
		FORWARD, BACKWARD
	};

public:
	ControlUnit(dsl::BaseProgram program);
	virtual ~ControlUnit();

	void execute();
	void execute_backwards();

	void runProgram(std::shared_ptr<Instruction> start, Direction d);
	void initialise(std::shared_ptr<Instruction> start, Direction d);
	void step();

	void errorInjectorEnable();
	void errorInjectorDisable();

	int getProgramCounterRelative();
	int getProgramCounterAbsolute();
	ControlUnit::Direction getDirectionCurrent();
	ControlUnit::Direction getDirectionIntended();
	std::shared_ptr<Instruction> getInstructionCurrent();
	std::shared_ptr<Instruction> getInstructionStart();
	std::shared_ptr<Instruction> getInstructionEnd();

	std::string toStr(Direction d);

private:

	enum class Result
	{
		CONTINUE, CHANGE_DIRECTION
	};

	void evaluate(std::shared_ptr<dsl::Instruction> ins, Direction dir);
	std::shared_ptr<dsl::Instruction> next(std::shared_ptr<dsl::Instruction> ins, Direction dir);
	Direction changeDirection(Direction dir);
	Result updateStates(std::shared_ptr<dsl::Instruction> ins, Direction dir);
	void updateErrorState(std::shared_ptr<dsl::Instruction> ins);

	void readMessages();
	bool handleMessage(dsl::MessageEnvelope & msg);
	template <typename T>
	bool handleMessage(dsl::MessageEnvelope & msg);
	bool handleMessage(dsl::MessagesError & msg);

	void attemptToResumeIntendedDirection();

	void coutState(std::shared_ptr<dsl::Instruction> ins, Direction dir);


private:

	dsl::BaseProgram _program;
	int _programCounterAbsolute;
	int _programCounterRelative;

	Direction _directionCurrent;
	Direction _directionIntended;

	std::shared_ptr<dsl::Instruction> _instructionCurrent;
	std::shared_ptr<dsl::Instruction> _instructionStart;
	std::shared_ptr<dsl::Instruction> _instructionEnd;

	dsl::MessagesContainer _msgContainer;
	dsl::ErrorHandleDeterministic _errHandler;

	//Controller flags
	bool _isInitialised;
	bool _isErrorMode;
	bool _isPostErrorMode;
	bool _occuredNewError;

// Debug
	dsl::debug::Debug dcu;

// Error injector
	boost::thread_group _tgroup;
	std::shared_ptr<dsl::ManualErrorInjecter> _errorInjector;



};





// ******************************************************************
// **** TEMPLATE IMPLEMENTATIONS ************************************
// ******************************************************************

template <typename T>
bool ControlUnit::handleMessage(dsl::MessageEnvelope &msg)
{
	T * a;
	a = dynamic_cast<T*>(&msg);
	if(a != 0)
		return handleMessage(*a);
	return false;
}

















} /* namespace dsl */

#endif /* CONTROLUNIT_HPP_ */
