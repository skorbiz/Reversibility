/*
 * CommandString.cpp
 *
 *  Created on: Apr 5, 2017
 *      Author: josl
 */

#include "CommandString.h"

#include "RegexIntepreter.h"

CommandString::CommandString(std::string line)
{
	command = RegexIntepreter::extract_command(line);
	args = RegexIntepreter::extract_args_list(line);
}

CommandString::~CommandString()
{

}

