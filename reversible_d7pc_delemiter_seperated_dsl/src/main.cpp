/*
 * main.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: josl
 */

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>

#include "File.h"
#include "CommandString.h"
#include "AssemblyProgram.h"
#include "CommandInterpreter.h"

int main(int argc, char **argv)
{
	std::cout << "Program start" << std::endl;

	std::vector<std::string> args(argv, argv+argc);
	std::cout << "Program inputs: " << std::endl;
	for(unsigned int i = 0; i < args.size(); i++)
		std::cout << i << ": " << args[i] << std::endl;


	bool do_backwards_execution = false;
	bool do_forwards_execution = false;
	for (size_t i = 1; i < args.size(); ++i)
	{
		if (args[i] == "-backward")
			do_backwards_execution = true;
		else if (args[i] == "-forward")
			do_forwards_execution = true;
		else if (args[i] == "-h" || true)
		{
			std::cout << "Usage is "
					"\n -h  : help "
					"\n -backwards "
					"\n" << std::endl;
			exit(-0);
		}
	}

	std::string filepath = ros::package::getPath("reversible_assembly") + "/src/d7pc_programs/";
	std::string filename_outershell 	=  "part1_outershell.txt";
	std::string filename_innerstructure =  "part2_innerstructure.txt";
	std::string filename_innerstructure_manual =  "part2_innerstructure_manual.txt";
	std::string filename_innerstructure_manual_insertion =  "part2_innerstructure_manual_insertion.txt";
	std::string filename_thermoelement 	=  "part3_thermoelement.txt";
	std::string filename_pin 			=  "part4_pin.txt";
	std::string filename_pin_manual_insertion =  "part4_pin_manual_insertion.txt";
	std::string filename_spring 		=  "part5_spring.txt";
	std::string filename_screwpart 		=  "part6_screwpart.txt";

	File file1(filepath, filename_outershell);
	File file2(filepath, filename_innerstructure);
	File file2_manual(filepath, filename_innerstructure_manual);
	File file2_manual_insertion(filepath, filename_innerstructure_manual_insertion);
	File file3(filepath, filename_thermoelement);
	File file4(filepath, filename_pin);
	File file4_manual_insertion(filepath, filename_pin_manual_insertion);
	File file5(filepath, filename_spring);
	File file6(filepath, filename_screwpart);

	std::vector<std::string> lines1 = file1.readLines();
	std::vector<std::string> lines2 = file2.readLines();
	std::vector<std::string> lines2_manual = file2_manual.readLines();
	std::vector<std::string> lines2_manual_insertion = file2_manual_insertion.readLines();
	std::vector<std::string> lines3 = file3.readLines();
	std::vector<std::string> lines4 = file4.readLines();
	std::vector<std::string> lines4_manual_insertion = file4_manual_insertion.readLines();
	std::vector<std::string> lines5 = file5.readLines();
	std::vector<std::string> lines6 = file6.readLines();

	std::vector<CommandString> commands1;
	std::vector<CommandString> commands2;
	std::vector<CommandString> commands2_manual;
	std::vector<CommandString> commands2_manual_insertion;
	std::vector<CommandString> commands3;
	std::vector<CommandString> commands4;
	std::vector<CommandString> commands4_manual_insertion;
	std::vector<CommandString> commands5;
	std::vector<CommandString> commands6;

	for(size_t i = 0; i < lines1.size(); i++)
	{
		CommandString cmd(lines1[i]);
		commands1.push_back(cmd);
	}
	for(size_t i = 0; i < lines2.size(); i++)
	{
		CommandString cmd(lines2[i]);
		commands2.push_back(cmd);
	}
	for(size_t i = 0; i < lines2_manual.size(); i++)
	{
		CommandString cmd(lines2_manual[i]);
		commands2_manual.push_back(cmd);
	}
	for(size_t i = 0; i < lines2_manual_insertion.size(); i++)
	{
		CommandString cmd(lines2_manual_insertion[i]);
		commands2_manual_insertion.push_back(cmd);
	}
	for(size_t i = 0; i < lines3.size(); i++)
	{
		CommandString cmd(lines3[i]);
		commands3.push_back(cmd);
	}
	for(size_t i = 0; i < lines4.size(); i++)
	{
		CommandString cmd(lines4[i]);
		commands4.push_back(cmd);
	}
	for(size_t i = 0; i < lines4_manual_insertion.size(); i++)
	{
		CommandString cmd(lines4_manual_insertion[i]);
		commands4_manual_insertion.push_back(cmd);
	}
	for(size_t i = 0; i < lines5.size(); i++)
	{
		CommandString cmd(lines5[i]);
		commands5.push_back(cmd);
	}
	for(size_t i = 0; i < lines6.size(); i++)
	{
		CommandString cmd(lines6[i]);
		commands6.push_back(cmd);
	}


	std::vector<CommandString> commands;
//	commands.insert(commands.end(), commands1.begin(), commands1.end());
//	commands.insert(commands.end(), commands2_manual.begin(), commands2_manual.end());
//	commands.insert(commands.end(), commands2_manual_insertion.begin(), commands2_manual_insertion.end());
//	commands.insert(commands.end(), commands2.begin(), commands2.end());
//	commands.insert(commands.end(), commands3.begin(), commands3.end());
	commands.insert(commands.end(), commands4.begin(), commands4.end());
//	commands.insert(commands.end(), commands4_manual_insertion.begin(), commands4_manual_insertion.end());
//	commands.insert(commands.end(), commands5.begin(), commands5.end());
//	commands.insert(commands.end(), commands6.begin(), commands6.end());

	std::shared_ptr<CommandInterpreter> interpreter = std::make_shared<CommandInterpreter>(argc, argv, "reversible_assembly");
	AssemblyProgram program(commands, interpreter);

	if(do_forwards_execution)
		program.execute_forward();
	else if(do_backwards_execution)
		program.execute_backward();
	else
	{
		for(int i = 0; i < 30; i++)
		{
			std::cout << dsl::color::BOLDCYAN << "############################# RUN: " << i << dsl::color::DEFAULT << std::endl;
			program.execute_forward_with_error_handler();
		}
	}


	std::cout << "program end" << std::endl;
	return 0;
}
