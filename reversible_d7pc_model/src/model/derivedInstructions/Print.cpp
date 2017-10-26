/*
 * Print.cpp
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#include <model/derivedInstructions/Print.h>
#include <model/MemoryModel.hpp>
#include <model/Variables.h>
#include <iostream>

namespace model {


Print::Print(std::string variabel_name) :
		variabel_name(variabel_name)
{
}

//Print::Print(Variable<std::string> text) :
//		text(text)
//{
//}

Print::~Print()
{
}


model::ExecutionResult23 Print::execute()
{
//	gen::dCommand << "executed print command: ";
//	std::cout << _textFrom << " -> " << _text << std::endl;
	assert(variables != nullptr);

	if(variables->varString.doContain(variabel_name))
	{
		std::cout << __PRETTY_FUNCTION__ << " " <<  variables->varString.getRef(variabel_name).get() << std::endl;
	}
	else
		std::cout << "Warning: Print::execute() did not find variable with name: " << variabel_name << std::endl;

//	std::cout << "\t \t \t \t \t from: " << _textFrom << std::endl;
	model::ExecutionResult23 r;
	return r;
}


ExecutionResult23 Print::executeBackwards()
{
	return execute();
}

} /* namespace model */
