/*
 * ControlUnit.h
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_CONTROLLER_CONTROLUNIT_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_CONTROLLER_CONTROLUNIT_H_

#include <debug/Debug.hpp>
#include <memory>
#include <string>

namespace model {
class Instruction;
} /* namespace model */

namespace model {

class ControlUnit
{

	enum class Direction
	{
		FORWARD, BACKWARD
	};

public:
	ControlUnit(std::shared_ptr<Instruction> rootInstruction);
	virtual ~ControlUnit();

	void execute();
	void execute_backwards();

	void runProgram(std::shared_ptr<Instruction> start, Direction d);
	void initialise(std::shared_ptr<Instruction> start, Direction d);
	void step();

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

	void evaluate(std::shared_ptr<Instruction> ins, Direction dir);
	std::shared_ptr<Instruction> next(std::shared_ptr<Instruction> ins, Direction dir);
	Direction changeDirection(Direction dir);
	Result updateStates(std::shared_ptr<Instruction> ins, Direction dir);
	void updateErrorState(std::shared_ptr<Instruction> ins);

	void attemptToResumeIntendedDirection();

	void coutState(std::shared_ptr<Instruction> ins, Direction dir);


private:

	std::shared_ptr<Instruction> _rootInstruction;
	int _programCounterAbsolute;
	int _programCounterRelative;

	Direction _directionCurrent;
	Direction _directionIntended;

	std::shared_ptr<Instruction> _instructionCurrent;
	std::shared_ptr<Instruction> _instructionStart;
	std::shared_ptr<Instruction> _instructionEnd;

	//Controller flags
	bool _isInitialised;
	bool _isErrorMode;
	bool _isPostErrorMode;
	bool _occuredNewError;

// Debug
	dsl::debug::Debug dcu;


};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_CONTROLLER_CONTROLUNIT_H_ */
