/*
 * ModelFactory.cpp
 *
 *  Created on: Sep 27, 2017
 *      Author: josl
 */



#include <ModelFactory.h>

#include "model/derivedInstructions/Assignment.h"
#include "model/derivedInstructions/Declaration.h"
#include "model/derivedInstructions/Nothing.h"
#include "model/derivedInstructions/PrepareFeeder.h"
#include "model/derivedInstructions/Print.h"
#include "model/derivedInstructions/RestartPoint.h"
#include "model/derivedInstructions/Update.h"
#include "model/D7pcWorkspace23.h"
#include "model/Identification23.h"
#include "model/Instruction.h"
#include "model/Variables.h"

namespace model {

ModelFactory::ModelFactory(std::shared_ptr<model::D7pcWorkspace23> workcell, std::shared_ptr<model::Variables> variable_container) :
		workcell(workcell),
		variable_container(variable_container)
{
}

ModelFactory::~ModelFactory()
{
}


void ModelFactory::connectInstructions(std::shared_ptr<model::Instruction> i1, std::shared_ptr<model::Instruction> i2)
{
	i1->setInstructionNext(i2);
	i2->setInstructionPrevious(i1);
}

std::shared_ptr<model::Instruction> ModelFactory::createInstruction()
{
	model::Identification23 id("t1",2,3,4);
	bool isReversible = false;
	bool isSymmetric = false;
	bool isSwapped = false;
	bool isUncallLayout = false;
	std::shared_ptr<model::Instruction> instruction = std::make_shared<model::Instruction>(id, isReversible, isSymmetric, isSwapped, isUncallLayout);
	return instruction;
}

std::shared_ptr<model::InstructionExecutable> ModelFactory::createPrint(std::string var)
{
	auto ins = std::make_shared<model::Print>(var);
	ins->setType("print");
	ins->setVariableContainer(variable_container);
	return ins;
}

std::shared_ptr<model::InstructionExecutable> ModelFactory::createPrepareFeeder(std::string var)
{
	auto ins = std::make_shared<model::PrepareFeeder>(nullptr,var);
	ins->setType("prepare_feeder");
	ins->setVariableContainer(variable_container);
	return ins;
}

std::shared_ptr<model::InstructionExecutable> ModelFactory::createRestartPoint()
{
	auto ins = std::make_shared<model::RestartPoint>();
	ins->setType("restart_point");
	ins->setVariableContainer(variable_container);
	return ins;
}

std::shared_ptr<model::InstructionExecutable> ModelFactory::createLocal(std::string varName, std::string type)
{
	auto ins = std::make_shared<model::Local>(workcell, varName,type);
	ins->setType("Local");
	ins->setVariableContainer(variable_container);
	return ins;
}

std::shared_ptr<model::InstructionExecutable> ModelFactory::createDelocal(std::string varName, std::string type)
{
	auto ins = std::make_shared<model::Delocal>(workcell, varName, type);
	ins->setType("Delocal");
	ins->setVariableContainer(variable_container);
	return ins;
}

std::shared_ptr<model::InstructionExecutable> ModelFactory::createAssignment(std::string name, std::string value)
{
	auto ins = std::make_shared<model::Assignment>(value, name);
	ins->setType("assignment");
	ins->setVariableContainer(variable_container);
	return ins;
}

std::shared_ptr<model::InstructionExecutable> ModelFactory::createUpdate(std::string varName, std::string var)
{
	auto ins = std::make_shared<model::Update>(varName);
	ins->setType("update");
	ins->setVariableContainer(variable_container);
	return ins;
}

std::shared_ptr<model::InstructionExecutable> ModelFactory::createNothing()
{
	auto ins = std::make_shared<model::Nothing>();
	ins->setType("Nothing");
	ins->setVariableContainer(variable_container);
	return ins;
}



} /* namespace model */
