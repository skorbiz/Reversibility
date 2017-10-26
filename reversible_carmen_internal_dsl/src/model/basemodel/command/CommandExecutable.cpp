/*
 * CommandExecutable.cpp
 *
 *  Created on: Nov 10, 2014
 *      Author: josl
 */

#include "CommandExecutable.hpp"

namespace dsl {

CommandExecutable::CommandExecutable()
{
	deb.setPrefix("[Command Executable] ");
	deb.setColor(dsl::color::GREEN);
}

CommandExecutable::~CommandExecutable()
{
}

void CommandExecutable::executeBackwards()
{
	std::cerr << "Tried to execute a command backwards but the command had no backwards counterpart" << std::endl;
	exit(-1);
}

void CommandExecutable::stateUpdate(dsl::State & state)
{
}

void CommandExecutable::stateUpdateSwapped(dsl::State & state)
{
}

std::string CommandExecutable::getArgumentForward() const
{
	return "";
}

std::string CommandExecutable::getArgumentBackward() const
{
	return "";
}


std::string CommandExecutable::getType()
{
	return "unkown/unoverloaded command type";
}



} /* namespace dsl */
