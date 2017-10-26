/*
 * main.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: josl
 */

#include <cstdlib>
#include <iostream>
#include <memory>

#include "../include/reversible_d7pc_model/ModelFactory.h"
#include "model/controller/ControlUnit.h"
#include "model/D7pcWorkspace23.h"
#include "model/Instruction.h"
#include "model/InstructionExecutable.h"
#include "model/Variables.h"


int main(int argc, char **argv)
{
	system("clear");
	system("clear");
	std::cout << "program start v. 000" << std::endl;

	std::shared_ptr<model::Variables> variable_container = std::make_shared<model::Variables>();
	std::shared_ptr<model::D7pcWorkspace23> workcell = std::make_shared<model::D7pcWorkspace23>();
	workcell->initEsentials();


	model::ModelFactory factory(workcell, variable_container);

	std::shared_ptr<model::InstructionExecutable> ie1_start = factory.createNothing();
	std::shared_ptr<model::InstructionExecutable> ie2_local = factory.createLocal("s","string");
	std::shared_ptr<model::InstructionExecutable> ie3_assignment = factory.createAssignment("s","my_string_local");
	std::shared_ptr<model::InstructionExecutable> ie4_print = factory.createPrint("s");
	std::shared_ptr<model::InstructionExecutable> ie5_assignment = factory.createAssignment("s","my_string_delocal");
	std::shared_ptr<model::InstructionExecutable> ie6_delocal = factory.createDelocal("s", "string");
	std::shared_ptr<model::InstructionExecutable> ie7_end = factory.createNothing();

	std::shared_ptr<model::Instruction> i1_start = factory.createInstruction();
	std::shared_ptr<model::Instruction> i2_local = factory.createInstruction();
	std::shared_ptr<model::Instruction> i3_assignment = factory.createInstruction();
	std::shared_ptr<model::Instruction> i4_print = factory.createInstruction();
	std::shared_ptr<model::Instruction> i5_assignment = factory.createInstruction();
	std::shared_ptr<model::Instruction> i6_delocal = factory.createInstruction();
	std::shared_ptr<model::Instruction> i7_end = factory.createInstruction();

	i1_start->setInstructionExecutable(ie1_start);
	i2_local->setInstructionExecutable(ie2_local);
	i3_assignment->setInstructionExecutable(ie3_assignment);
	i4_print->setInstructionExecutable(ie4_print);
	i5_assignment->setInstructionExecutable(ie5_assignment);
	i6_delocal->setInstructionExecutable(ie6_delocal);
	i7_end->setInstructionExecutable(ie7_end);

	factory.connectInstructions(i1_start, i2_local);
	factory.connectInstructions(i2_local, i3_assignment);
	factory.connectInstructions(i3_assignment, i4_print);
	factory.connectInstructions(i4_print, i5_assignment);
	factory.connectInstructions(i5_assignment, i6_delocal);
	factory.connectInstructions(i6_delocal, i7_end);

	model::ControlUnit control(i1_start);
	control.execute();
	control.execute_backwards();




}
