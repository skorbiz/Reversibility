/*
 * ControlUnit.cpp
 *
 *  Created on: Dec 1, 2014
 *      Author: josl
 */

#include "ControlUnit.hpp"

namespace dsl
{

ControlUnit::ControlUnit(dsl::BaseProgram program) :
	_program(program),
	_programCounterAbsolute(0),
	_programCounterRelative(0),
	_directionCurrent(Direction::FORWARD),
	_directionIntended(Direction::FORWARD),
	_instructionCurrent(nullptr),
	_instructionStart(_program.getStart()),
	_instructionEnd(_program.getEnd()),
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
	errorInjectorDisable();
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

	_program.initInstructions(&_msgContainer);
}



void ControlUnit::step()
{
	assert(_isInitialised);

	Result r = Result::CONTINUE;
	std::shared_ptr<dsl::Instruction> i = _instructionCurrent;
	Direction d = _directionCurrent;

	evaluate(i,d);
	readMessages();
	r = updateStates(i, d);

	if(r == Result::CONTINUE)
		_instructionCurrent = next(i,d);
	else
		_directionCurrent = changeDirection(d);
}



void ControlUnit::errorInjectorEnable()
{
	_errorInjector = std::make_shared<ManualErrorInjecter>(&_msgContainer);
	_tgroup.create_thread( boost::bind( &ManualErrorInjecter::run, _errorInjector) );
}

void ControlUnit::errorInjectorDisable()
{
	if(_errorInjector != nullptr)
		_errorInjector->stop();
	_tgroup.join_all();
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

void ControlUnit::evaluate(std::shared_ptr<dsl::Instruction> ins, Direction dir)
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

std::shared_ptr<dsl::Instruction> ControlUnit::next(std::shared_ptr<dsl::Instruction> ins, Direction dir)
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

ControlUnit::Result ControlUnit::updateStates(std::shared_ptr<dsl::Instruction> ins, Direction dir)
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

void ControlUnit::updateErrorState(std::shared_ptr<dsl::Instruction> ins)
{
	_errHandler.update(ins);

	if( _errHandler.isInErrorMode() == false )
	{
		_isErrorMode = false;
		_isPostErrorMode = true;
	}
}

void ControlUnit::readMessages()
{
	while( _msgContainer.containerIsEmpty() == false )
	{
		dsl::MessageEnvelope &msg = *(_msgContainer.popMessage());
		handleMessage(msg);
	}
}

bool ControlUnit::handleMessage(dsl::MessageEnvelope & msg)
{
	bool m1 = handleMessage<dsl::MessagesError>(msg);
//	bool m2 = handleMessage<dsl::MessagesError>(msg);
//	return ( m1 || m2 );
	return m1;
}

bool ControlUnit::handleMessage(dsl::MessagesError & msg)
{
	_errHandler.update(msg);
	_isErrorMode = true;
	_isPostErrorMode = false;
	_occuredNewError = true;
	return true;
}


void ControlUnit::coutState(std::shared_ptr<dsl::Instruction> i, Direction d)
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

} /* namespace dsl */
