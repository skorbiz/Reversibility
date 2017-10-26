/*
 * CommandString.h
 *
 *  Created on: Apr 5, 2017
 *      Author: josl
 */

#ifndef REVERSIBLE_ASSEMBLY_SRC_COMMANDSTRING_H_
#define REVERSIBLE_ASSEMBLY_SRC_COMMANDSTRING_H_

#include <iostream>
#include <vector>

class CommandString
{

public:
	CommandString(std::string line);
	virtual ~CommandString();

	std::string command;
	std::vector<std::string> args;

};

#endif /* REVERSIBLE_ASSEMBLY_SRC_COMMANDSTRING_H_ */
