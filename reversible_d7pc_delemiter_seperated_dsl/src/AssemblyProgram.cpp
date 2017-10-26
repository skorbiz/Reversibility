/*
 * AssemblyProgram.cpp
 *
 *  Created on: Apr 5, 2017
 *      Author: josl
 */

#include "AssemblyProgram.h"

AssemblyProgram::AssemblyProgram(std::vector<CommandString> commands, std::shared_ptr<CommandInterpreter> interpreter) :
	commands(commands),
	interpreter(interpreter),
	line(0)
{
}

AssemblyProgram::~AssemblyProgram()
{
}

void AssemblyProgram::execute_forward()
{
	for(size_t i = 0; i < commands.size(); i++)
	{
		std::cout << i+1 << " " << commands[i].command << " : ";
		for(size_t j = 0; j < commands[i].args.size(); j++)
			std::cout << commands[i].args[j] << " ";
		std::cout << std::endl;

		interpreter->execute(commands[i]);
	}
}

void AssemblyProgram::execute_backward()
{
	for(size_t i = commands.size()-1; i >= 0; i--)
	{
		std::cout << i+1 << " " << commands[i].command << " : ";
		for(size_t j = 0; j < commands[i].args.size(); j++)
			std::cout << commands[i].args[j] << " ";
		std::cout << std::endl;

		bool reverse = true;
		interpreter->execute(commands[i], reverse);
	}
}





void AssemblyProgram::execute_forward_with_error_handler()
{
	line = 0;
	while(line < commands.size())
	{
		std::cout << line+1 << " " << commands[line].command << " : ";
		for(size_t j = 0; j < commands[line].args.size(); j++)
			std::cout << commands[line].args[j] << " ";
		std::cout << std::endl;

		auto result = interpreter->execute(commands[line]);

		if(result.was_failure)
		{
			line--;
			execute_backward_to_reset(0);
		}
		line++;
	}
}

void AssemblyProgram::execute_backward_to_reset(int resets_to_pass)
{
	while(line >= 0 && resets_to_pass >= 0)
	{
		std::cout << line+1 << " " << commands[line].command << " : ";
		for(size_t j = 0; j < commands[line].args.size(); j++)
			std::cout << commands[line].args[j] << " ";
		std::cout << std::endl;

		auto result = interpreter->execute(commands[line], true);
		line--;

		if(commands[line].command == "restart_point")
			resets_to_pass--;
	}
}


