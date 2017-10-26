/*
 * EmptyLinker.hpp
 *
 *  Created on: Jan 2, 2015
 *      Author: josl
 */

#ifndef EMPTYLINKER_HPP_
#define EMPTYLINKER_HPP_

#include <iostream>
#include <../src/model/basemodel/command/CommandExecutable.hpp>

namespace dsl
{

class EmptyLinker  : public dsl::CommandExecutable
{

public:
	EmptyLinker(std::string text);
	virtual ~EmptyLinker();

	virtual void execute();
	virtual void executeBackwards();
	virtual std::string getType();

private:
	std::string _text;

};

} /* namespace dsl */

#endif /* EMPTYLINKER_HPP_ */
