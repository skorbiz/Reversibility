/*
 * Declaration.cpp
 *
 *  Created on: Jun 20, 2017
 *      Author: josl
 */

#include <model/derivedInstructions/Declaration.h>
#include <model/D7pcWorkspace23.h>
#include <model/MemoryModel.hpp>
#include <model/Variable.h>
#include <model/Variables.h>
#include <rw/kinematics/State.hpp>
#include <iostream>
#include <assert.h>

namespace model {

Declaration::Declaration(std::shared_ptr<D7pcWorkspace23> wc, std::string varName, std::string varType) :
		wc(wc),
		varName(varName),
		varType(varType)
{
}

Declaration::~Declaration()
{
}

ExecutionResult23 Declaration::create(std::shared_ptr<Variables> variables)
{
	assert(variables != nullptr);
	if(varType == "string")
	{
		model::Variable<std::string> var(varName, "");
		variables->varString.save(varName, var);
	}
	else if(varType == "State")
	{
		model::Variable<rw::kinematics::State> var(varName, wc->getDefaultState());
		variables->varState.save(varName, var);
	}
	else
		std::cout << "Warning: Declaration::create() failed to declare variable with name " << varName << " and type " << varType << std::endl;

	ExecutionResult23 r;
	return r;
}

ExecutionResult23 Declaration::remove(std::shared_ptr<Variables> variables)
{
	assert(variables != nullptr);
	if(varType == "string")
	{
		model::Variable<std::string> var(varName, "");
		variables->varString.erase(varName);
	}
	else if(varType == "State")
	{
		model::Variable<rw::kinematics::State> var(varName, wc->getDefaultState());
		variables->varState.erase(varName);
	}
	else
		std::cout << "Warning: Declaration::remove() failed to remove variable with name " << varName << " and type " << varType << std::endl;

	ExecutionResult23 r;
	return r;
}



Local::Local(std::shared_ptr<D7pcWorkspace23> wc, std::string varName, std::string varType):Declaration(wc, varName, varType){}
Local::~Local(){}
ExecutionResult23 Local::execute(){return create(variables);}
ExecutionResult23 Local::executeBackwards(){return remove(variables);}

Delocal::Delocal(std::shared_ptr<D7pcWorkspace23> wc, std::string varName, std::string varType):Declaration(wc, varName, varType){}
Delocal::~Delocal(){}
ExecutionResult23 Delocal::execute(){return remove(variables);}
ExecutionResult23 Delocal::executeBackwards(){return create(variables);}


} /* namespace model */
