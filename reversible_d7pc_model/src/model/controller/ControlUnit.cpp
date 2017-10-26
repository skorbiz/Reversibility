/*
 * ControlUnit.cpp
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#include <assert.h>
#include <dsl_common/color/colors.hpp>
#include <model/controller/ControlUnit.h>
#include <model/ExecutionResult23.h>
#include <model/Identification23.h>
#include <model/Instruction.h>
#include <iostream>
#include <memory>
#include <string>

namespace model {

ControlUnit::ControlUnit(std::shared_ptr<Instruction> rootInstruction) :
	_rootInstruction(rootInstruction),
	_programCounterAbsolute(0),
	_programCounterRelative(0),
	_directionCurrent(Direction::FORWARD),
	_directionIntended(Direction::FORWARD),
	_instructionCurrent(nullptr),
	_instructionStart(rootInstruction),
	_instructionEnd(rootInstruction),
	_isInitialised(false),
	_isErrorMode(false),
	_isPostErrorMode(false),
	_occuredNewError(false)
{
	dcu.setColor(dsl::color::BLUE);
//	dcu.disable();
}

ControlUnit::~ControlUnit()
{
}

void ControlUnit::execute()
{
	dcu << "Executing program forwards " << std::endl;
	runProgram(_instructionStart, Direction::FORWARD);
}

void ControlUnit::execute_backwards()
{
	dcu << "Executing program backwards" << std::endl;
	runProgram(_instructionEnd, Direction::BACKWARD);
}

void ControlUnit::runProgram(std::shared_ptr<Instruction> start, Direction d)
{
	initialise(start, d);

	while(_instructionCurrent != nullptr)
	{
		coutState(_instructionCurrent,_directionCurrent);
		step();
	}
}

void ControlUnit::initialise(std::shared_ptr<Instruction> start, Direction d)
{
	_isInitialised = true;

	_instructionCurrent = start;
	_directionIntended = d;

	_isErrorMode = false;
	_isPostErrorMode = false;
	_occuredNewError = false;
}



void ControlUnit::step()
{
	assert(_isInitialised);

	Result r = Result::CONTINUE;
	std::shared_ptr<Instruction> i = _instructionCurrent;
	Direction d = _directionCurrent;

	evaluate(i,d);
	r = updateStates(i, d);

	if(r == Result::CONTINUE)
		_instructionCurrent = next(i,d);
	else
		_directionCurrent = changeDirection(d);
}


int ControlUnit::getProgramCounterRelative()
{
	return _programCounterRelative;
}

int ControlUnit::getProgramCounterAbsolute()
{
	return _programCounterAbsolute;
}

ControlUnit::Direction ControlUnit::getDirectionCurrent()
{
	return _directionCurrent;
}

ControlUnit::Direction ControlUnit::getDirectionIntended()
{
	return _directionIntended;
}

std::shared_ptr<Instruction> ControlUnit::getInstructionCurrent()
{
	return _instructionCurrent;
}

std::shared_ptr<Instruction> ControlUnit::getInstructionStart()
{
	return _instructionStart;
}

std::shared_ptr<Instruction> ControlUnit::getInstructionEnd()
{
	return _instructionEnd;
}

std::string ControlUnit::toStr(Direction d)
{
	if(d == Direction::FORWARD)
		return "Forward";
	return "Backward";
}



// **********************************
// *** PRIVATE FUNCTIONS

void ControlUnit::evaluate(std::shared_ptr<Instruction> ins, Direction dir)
{
	_programCounterAbsolute++;
	if(dir == Direction::FORWARD)
	{
		ins->executeForward();
		_programCounterRelative++;
	}
	else
	{
		ins->executeBackward();
		_programCounterRelative--;
	}
}

std::shared_ptr<Instruction> ControlUnit::next(std::shared_ptr<Instruction> ins, Direction dir)
{
	if(dir == Direction::FORWARD)
		return ins->getInstructionNext();
	return ins->getInstructionPrevious();
}

ControlUnit::Direction ControlUnit::changeDirection(Direction dir)
{
	if(dir == Direction::BACKWARD)
		return Direction::FORWARD;
	return Direction::BACKWARD;
}

ControlUnit::Result ControlUnit::updateStates(std::shared_ptr<Instruction> ins, Direction dir)
{
	if(_isErrorMode)
		updateErrorState(ins);

	if(_occuredNewError == true){
		_occuredNewError = false;
		return Result::CHANGE_DIRECTION;
	}

	if(_isPostErrorMode == false)
		return Result::CONTINUE;

	if(dir == _directionIntended){
		_isPostErrorMode = false;
		return Result::CONTINUE;
	}

	if(_directionIntended == Direction::FORWARD)
		if(ins->isForwardExecutable()){
			_isPostErrorMode = false;
			return Result::CHANGE_DIRECTION;
		}

	if(_directionIntended == Direction::BACKWARD)
		if(ins->isBackwardExecutable()){
			_isPostErrorMode = false;
			return Result::CHANGE_DIRECTION;
		}

	return Result::CONTINUE;
}

void ControlUnit::updateErrorState(std::shared_ptr<Instruction> ins)
{
//	if( _errHandler.isInErrorMode() == false )
//	{
//		_isErrorMode = false;
//		_isPostErrorMode = true;
//	}
}



void ControlUnit::coutState(std::shared_ptr<Instruction> i, Direction d)
{
	int depth = i->getIDClass().getDepth();
	std::string depthS = "";
	for(int a = 0; a < depth; a++ )
		depthS += " ";

	std::string info = "";
	if(i->isUncallLayoutDEBUG())
		info += "U ";
	if(i->isSwappedLayoutDEBUG())
		info += " S";
	if(!i->isReversibleDEBUG())
		info += " N";

	dcu << "Program counter: " << _programCounterRelative << " / " << _programCounterAbsolute << " : ";
	dcu << toStr(d) << " : ";
	dcu << depthS;
	dcu << i->getID() << " : " << i->getType() << " : ";
	dcu << info << " : ";
//	dcu << (_isErrorMode ? dsl::color::red("error mode") : "");
//	dcu << (_isPostErrorMode ? dsl::color::red("post error mode") : "");
	dcu << std::endl;

}


} /* namespace model */
