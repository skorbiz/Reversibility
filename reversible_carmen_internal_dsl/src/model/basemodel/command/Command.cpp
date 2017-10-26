/*
 * Command.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: josl
 */

#include "Command.hpp"

namespace dsl
{

Command::Command(dsl::Identification id, std::shared_ptr<dsl::CommandExecutable> cmdExecutable, bool isReversible, bool isSymmetric, bool isSwapped, bool isUncallLayout) :
		Instruction(id, isReversible, isSymmetric, isSwapped, isUncallLayout),
		_executableCommand(cmdExecutable)
{
}

Command::~Command()
{
}

void Command::doExecuteForward()
{
	if(_executableCommand == nullptr)
	{
		std::cerr << "Attempted to execute dsl::command with nullptr" << std::endl;
		exit(-1);
	}

	_executableCommand->execute();
}

void Command::doExecuteBackward()
{
	if(_executableCommand == nullptr)
	{
		std::cerr << "Attempted to backwards_execute dsl::command with nullptr" << std::endl;
		exit(-1);
	}
	_executableCommand->executeBackwards();
 }

std::string Command::getType()
{
	return _executableCommand->getType();
}

void Command::stateUpdate(dsl::State & state)
{
	debIB << "state update in: " << getID() << " of type: " << getType() << std::endl;
	_executableCommand->stateUpdate(state);
}

void Command::stateUpdateSwapped(dsl::State & state)
{
	debIB << "state update in: " << getID() << " of type: " << getType() << " (swapped update)" << std::endl;
	_executableCommand->stateUpdateSwapped(state);
}

std::string Command::getArgumentForward() const
{
	return _executableCommand->getArgumentForward();
}

std::string Command::getArgumentBackward() const
{
	return _executableCommand->getArgumentBackward();
}



} /* namespace dsl */
