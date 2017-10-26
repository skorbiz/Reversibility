/*
 * Instruction.cpp
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#include <assert.h>
#include <color/colors.hpp>
#include <model/Instruction.h>
#include <model/InstructionExecutable.h>
#include <cstdlib>
#include <ctime>
#include <iostream>

namespace model {

Instruction::Instruction(Identification23 id, bool isReversible, bool isSymmetric, bool isSwapped, bool isUncallLayout) :
	_id(id),
	_instructionNext(nullptr),
	_instructionPrevious(nullptr),
	_isReversible(isReversible),
	_isSwapped(isSwapped),
	_isUncallLayout(isUncallLayout),
	_lastExecutionWasForward(false)
{
	deb.setPrefix("[Instruction]");
	deb.disable();

	debIB.setPrefix("[Instruction :: build process]");
	debIB.setColorPrefix(dsl::color::MAGENTA);
}

Instruction::~Instruction()
{
}



// **********************************
// *** Construction functions

void Instruction::setInstructionExecutable(std::shared_ptr<InstructionExecutable> executable)
{
	_executable = executable;
}

void Instruction::setInstructionNext(std::shared_ptr<Instruction> ins)
{
	_instructionNext = ins;
}

void Instruction::setInstructionPrevious(std::shared_ptr<Instruction> ins)
{
	_instructionPrevious= ins;
}

std::shared_ptr<Instruction> Instruction::getInstructionNext() const
{
	return _instructionNext;
}

std::shared_ptr<Instruction> Instruction::getInstructionPrevious() const
{
	return _instructionPrevious;
}

std::string Instruction::getID()
{
	return _id.toString();
}

Identification23 Instruction::getIDClass()
{
	return _id;
}

// **********************************
// *** Debug functions

bool Instruction::isUncallLayoutDEBUG() const
{
	return _isUncallLayout;
}
bool Instruction::isSwappedLayoutDEBUG() const
{
	return _isSwapped;
}
bool Instruction::isReversibleDEBUG() const
{
	return _isReversible;
}

std::string Instruction::getType()
{
	if(_executable != nullptr)
		return _executable->getType();
	return "unspecified";
}
std::string Instruction::getArgumentForward() const
{
	return "";
}
std::string Instruction::getArgumentBackward() const
{
	return "";
}


// **********************************
// *** look-ahead-funtions init functions


// *******************************
// *** Error functions


// **********************************
// *** Execution functions
ExecutionResult23 Instruction::executeForward()
{
	toDebug("forward");
	return (_isSwapped ? executeBackwardLogic() : executeForwardLogic());
}

ExecutionResult23 Instruction::executeBackward()
{
	toDebug("backward");
	return (_isSwapped ? executeForwardLogic() : executeBackwardLogic());
}

ExecutionResult23 Instruction::executeForwardLogic()
{
	assert(_executable != nullptr);
	return _executable->execute();
}

ExecutionResult23 Instruction::executeBackwardLogic()
{
	isReversibelOrDie();
	assert(_executable != nullptr);
	return _executable->executeBackwards();
}

bool Instruction::isForwardExecutable()
{
	if(	_isSwapped )
		return _isReversible;
	return true;
}

bool Instruction::isBackwardExecutable()
{
	if( _isSwapped )
		return true;
	else
		return _isReversible;
}

void Instruction::isReversibelOrDie()
{
	if(_isReversible)
		return;

	std::cerr << "Tried to reverse Instruction which was not reversible." << std::endl;
	std::cerr <<  "either it is never reversible or has not been forward executed." << std::endl;
	std::cerr <<  "instruction id was: " << getID() << std::endl;
	exit(-1);
}

void Instruction::toDebug(std::string executionDirection)
{
	deb << "INSRTUCTION INFO" << std::endl;
	deb << "ID: "<<  _id << std::endl;
	deb << "TYPE: " << getType() << std::endl;
	deb << "Execution direction: " << executionDirection << std::endl;
	deb << "LastExecutionWasForward: "<< _lastExecutionWasForward << std::endl;
	deb << "Time: ";
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    deb << (now->tm_year + 1900) << '-'
         << (now->tm_mon + 1) << '-'
         <<  now->tm_mday << " :: "
         << now->tm_hour << ":"
         << now->tm_min << ":"
         << now->tm_sec
         << std::endl;
    deb << "Instruction next: ";
    if(_instructionNext != nullptr)
    	deb << _instructionNext->getID() << std::endl;
    else
    	deb << "nullptr";
    deb << "Instruction prev: ";
    if(_instructionPrevious != nullptr)
    	deb << _instructionPrevious->getID() << std::endl;
    else
    	deb << "nullptr";
    deb << std::endl;
}



} /* namespace model */
