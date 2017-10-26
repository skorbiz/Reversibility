/*
 * EmptyLinker.cpp
 *
 *  Created on: Jan 2, 2015
 *      Author: josl
 */

#include "EmptyLinker.hpp"

namespace dsl
{

EmptyLinker::EmptyLinker(std::string text) :
		_text(text)
{
}

EmptyLinker::~EmptyLinker()
{
}

void EmptyLinker::execute()
{
//	gen::dCommand << _text << std::endl;
//	gen::dCommand << "Executed empty linker command" << std::endl;
}

void EmptyLinker::executeBackwards()
{
//	gen::dCommand << _text << std::endl;
//	gen::dCommand << "Reverse Executed empty linker command" << std::endl;
}

std::string EmptyLinker::getType()
{
	return "empty linker";
}

} /* namespace dsl */
