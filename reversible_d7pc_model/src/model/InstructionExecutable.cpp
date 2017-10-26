/*
 * InstructionExecutable.cpp
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#include <color/colors.hpp>
#include <model/ExecutionResult23.h>
#include <model/InstructionExecutable.h>
#include <cstdlib>
#include <iostream>

namespace model {

InstructionExecutable::InstructionExecutable() :
		type("unkown/unoverloaded command type")
{
	deb.setPrefix("[Command Executable] ");
	deb.setColor(dsl::color::GREEN);
}

InstructionExecutable::~InstructionExecutable()
{
}

void InstructionExecutable::setVariableContainer(std::shared_ptr<Variables> container)
{
	variables = container;
}

void InstructionExecutable::addArg(std::string arg)
{
	args.push_back(arg);
}


ExecutionResult23 InstructionExecutable::execute()
{
	std::cerr << __PRETTY_FUNCTION__;
	std::cerr << " - Error: Tried to execute a command of basetype" << std::endl;
	exit(-1);
}

ExecutionResult23 InstructionExecutable::executeBackwards()
{
	std::cerr << __PRETTY_FUNCTION__;
	std::cerr << " - Error: Tried to execute a command of basetype" << std::endl;
	exit(-1);
}

std::string InstructionExecutable::getArgumentForward() const
{
	return "";
}

std::string InstructionExecutable::getArgumentBackward() const
{
	return "";
}


std::string InstructionExecutable::getType()
{
	return type;
}

void InstructionExecutable::setType(std::string aType)
{
	type = aType;
}


} /* namespace model */
