/*


 * Instruction.cpp
 *
 *  Created on: Jan 9, 2015
 *      Author: josl
 */

#include "Instruction.hpp"

namespace dsl
{

Instruction::Instruction(dsl::Identification id, bool isReversible, bool isSymmetric, bool isSwapped, bool isUncallLayout) :
	_id(id),
	_instructionNext(nullptr),
	_instructionPrevious(nullptr),
	_isReversible(isReversible),
	_isSwapped(isSwapped),
	_isUncallLayout(isUncallLayout),
	_lastExecutionWasForward(false),
//	_visited(false),
	_messagesContainer(nullptr)
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

void Instruction::setInstructionNext(std::shared_ptr<Instruction> ins)
{
	this->_instructionNext = ins;
}

void Instruction::setInstructionPrevious(std::shared_ptr<Instruction> ins)
{
	this->_instructionPrevious= ins;
}

std::shared_ptr<Instruction> Instruction::getInstructionNext() const
{
	return this->_instructionNext;
}

std::shared_ptr<Instruction> Instruction::getInstructionPrevious() const
{
	return this->_instructionPrevious;
}

std::string Instruction::getID()
{
	return _id.toString();
}

dsl::Identification Instruction::getIDClass()
{
	return _id;
}

// **********************************
// *** Debug functions

bool dsl::Instruction::isUncallLayoutDEBUG() const
{
	return _isUncallLayout;
}
bool dsl::Instruction::isSwappedLayoutDEBUG() const
{
	return _isSwapped;
}
bool dsl::Instruction::isReversibleDEBUG() const
{
	return _isReversible;
}

std::string Instruction::getType()
{
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

void Instruction::stateUpdate(dsl::State & state)
{
}

void Instruction::stateUpdateSwapped(dsl::State & state)
{
}

dsl::State Instruction::initStateForward(dsl::State state)
{
	if(_isSwapped)
		stateUpdateSwapped(state);
	else
		stateUpdate(state);
	return state;
}

dsl::State Instruction::initStateReverse(dsl::State state)
{
	if(_isSwapped)
		stateUpdate(state);
	else
		stateUpdateSwapped(state);
	return state;
}


// *******************************
// *** Error functions

void Instruction::initMessageContainer(dsl::MessagesContainer * container)
{
	_messagesContainer = container;
}

void Instruction::postMessageToController(dsl::MessageEnvelope * msgToController)
{
	if(_messagesContainer == nullptr)
	{
		std::cerr << "_messagesContainer had not been initialised in instruction before posting." << std::endl;
		exit(-1);
	}

	_messagesContainer->pushMessage(msgToController);
}


// **********************************
// *** Execution functions
void Instruction::executeForward()
{
	toDebug("forward");
	_isSwapped ? executeBackwardLogic() : executeForwardLogic();
}

void Instruction::executeBackward()
{
	toDebug("backward");
	_isSwapped ? executeForwardLogic() : executeBackwardLogic();
}

void Instruction::executeForwardLogic()
{
	doExecuteForward();
}

void Instruction::executeBackwardLogic()
{
	isReversibelOrDie();
	doExecuteBackward();
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



} /* namespace dsl */



// **********************************
// *** look-ahead-funtions functions
//void Instruction::initStateAttributes()
//{
//	dsl::State state;
//	recursivlyClearVisited();
//	recursivlyPassForwardState(state, true);
//	recursivlyClearVisited();
//	recursivlyPassBackwardState();
//}
//
//void Instruction::initInstructions(dsl::MessagesContainer * container)
//{
//	dsl::InstructionGraph graph(this);
//
//	for(auto imap: graph.getAll())
//		imap->initMessageContainer(container);
//}
//
//
//void Instruction::initStateRecursivly(dsl::State state, dsl::InstructionGraph & graph)
//{
//}
//
//void Instruction::recursivlyClearVisited()
//{
//	if(_visited == false)
//		return;
//	_visited = false;
//
//	if(_instructionNext != nullptr)
//		_instructionNext->recursivlyClearVisited();
//
//	if(_instructionPrevious != nullptr)
//		_instructionPrevious->recursivlyClearVisited();
//}
//
//void Instruction::recursivlyPassForwardState(dsl::State & state, bool isPassedForward)
//{
//	if(_visited)
//		return;
//	_visited = true;
//
//	if(_isUncallLayout)
//		stateUpdateSwapped(state);
//	else
//		stateUpdate(state);
//
//	dsl::State stateNextCpy;
//	dsl::State statePrevCpy;
//
//	if(isPassedForward)
//		stateNextCpy = state;
//	else
//		statePrevCpy = state;
//
//	if(_instructionNext != nullptr)
//		_instructionNext->recursivlyPassForwardState(stateNextCpy, true);
//
//	if(_instructionPrevious != nullptr)
//		_instructionPrevious->recursivlyPassForwardState(statePrevCpy, false);
//}
//
//dsl::State Instruction::recursivlyPassBackwardState()
//{
//	if(_visited)
//	{
//		dsl::State emptyState;
//		return emptyState;
//	}
//	_visited = true;
//
//	dsl::State stateForward;
//	if(_instructionNext != nullptr)
//		stateForward = _instructionNext->recursivlyPassBackwardState();
//
//	dsl::State stateBackward;
//	if(_instructionPrevious != nullptr)
//		stateBackward = _instructionPrevious->recursivlyPassBackwardState();
//
//
//
//	if( _isUncallLayout )
//	{
//		dsl::State state;
//		state = _isSwapped ? stateForward : stateBackward;
//		stateUpdate(state);
//		return state;
//	}
//
//	dsl::State emptyState;
//	return emptyState;
//}
