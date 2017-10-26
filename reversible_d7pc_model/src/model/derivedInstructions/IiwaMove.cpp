/*
 * IiwaMove.cpp
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#include <model/derivedInstructions/IiwaMove.h>
#include <iostream>

namespace model {

IiwaMove::IiwaMove(std::string text, std::string text_from) :
				_text(text),
				_textFrom(text_from)
{
}

IiwaMove::~IiwaMove()
{
}

ExecutionResult23 IiwaMove::execute()
{
//	gen::dCommand << "executed print command: ";
//	std::cout << _textFrom << " -> " << _text << std::endl;
	std::cout << _text << std::endl;
//	std::cout << "\t \t \t \t \t from: " << _textFrom << std::endl;
	ExecutionResult23 r;
	return r;

}


ExecutionResult23 IiwaMove::executeBackwards()
{
//	gen::dCommand << "executed print command backwards: ";
	std::cout << _text << " -> " << _textFrom << std::endl;
	std::cout << _text << std::endl;
	std::cout << _textFrom << "\t \t \t \t \t \t org: " << _text << std::endl;
	std::cout << _text << " -> " << _textFrom << std::endl;
	ExecutionResult23 r;
	return r;

}

std::string IiwaMove::getType()
{
	return "IiwaMove";
}

} /* namespace model */
