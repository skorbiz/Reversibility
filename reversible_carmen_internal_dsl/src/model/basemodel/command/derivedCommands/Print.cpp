/*
 * Print.cpp
 *
 *  Created on: Jan 6, 2015
 *      Author: josl
 */

#include "Print.hpp"

namespace dsl {

Print::Print(std::string text) :
	_text(text)
{
}

Print::~Print()
{
}

void Print::execute()
{
//	gen::dCommand << "executed print command: ";
//	std::cout << _textFrom << " -> " << _text << std::endl;
	std::cout << _text << std::endl;
//	std::cout << "\t \t \t \t \t from: " << _textFrom << std::endl;

}


void Print::executeBackwards()
{
//	gen::dCommand << "executed print command backwards: ";
//	std::cout << _text << " -> " << _textFrom << std::endl;
//	std::cout << _text << std::endl;
//	std::cout << _textFrom << "\t \t \t \t \t \t org: " << _text << std::endl;
//	std::cout << _text << " -> " << _textFrom << std::endl;


}

void Print::stateUpdate(dsl::State & state)
{
	_textFrom = state.getText();
	state.update(_text);
}

void Print::stateUpdateSwapped(dsl::State & state)
{
	state.update(_textFrom);
}



std::string Print::getType()
{
	return "print";
}

} /* namespace dsl */


