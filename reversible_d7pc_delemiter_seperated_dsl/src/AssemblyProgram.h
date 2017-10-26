/*
 * AssemblyProgram.h
 *
 *  Created on: Apr 5, 2017
 *      Author: josl
 */

#ifndef REVERSIBLE_ASSEMBLY_SRC_ASSEMBLYPROGRAM_H_
#define REVERSIBLE_ASSEMBLY_SRC_ASSEMBLYPROGRAM_H_

#include <vector>
#include <memory>
#include "CommandString.h"
#include "CommandInterpreter.h"


class AssemblyProgram
{

public:
	AssemblyProgram(std::vector<CommandString> commands, std::shared_ptr<CommandInterpreter> interpreter);
	virtual ~AssemblyProgram();

	void execute_forward();
	void execute_backward();


	void execute_forward_with_error_handler();
	void execute_backward_to_reset(int resets_to_pass);



private:
	std::vector<CommandString> commands;
	std::shared_ptr<CommandInterpreter> interpreter;
	int line;

};

#endif /* REVERSIBLE_ASSEMBLY_SRC_ASSEMBLYPROGRAM_H_ */
