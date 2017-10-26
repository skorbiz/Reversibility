/*
 * Print.hpp
 *
 *  Created on: Jan 6, 2015
 *      Author: josl
 */

#ifndef PRINT_HPP_
#define PRINT_HPP_

#include <iostream>
#include <../src/model/basemodel/command/CommandExecutable.hpp>

namespace dsl
{

class Print : public dsl::CommandExecutable
{

public:
	Print(std::string text);
	virtual ~Print();

	virtual void execute();
	virtual void executeBackwards();
	void stateUpdate(dsl::State & state);
	void stateUpdateSwapped(dsl::State & state);
	std::string getType();

private:
	std::string _text;
	std::string _textFrom;
};

} /* namespace dsl */

#endif /* PRINT_HPP_ */

