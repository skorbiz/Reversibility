/*
 * CommandExecutable.hpp
 *
 *  Created on: Nov 10, 2014
 *      Author: josl
 */

#ifndef COMMANDEXECUTABLE_HPP_
#define COMMANDEXECUTABLE_HPP_

#include <iostream>
#include <rw/math.hpp>
#include <../src/model/basemodel/State.hpp>
#include <debug/Debug.hpp>
#include <color/colors.hpp>

namespace dsl
{

class CommandExecutable {
public:
	CommandExecutable();
	virtual ~CommandExecutable();
	virtual void execute() = 0;
	virtual void executeBackwards();
	virtual void stateUpdate(dsl::State & state);
	virtual void stateUpdateSwapped(dsl::State & state);
	virtual std::string getArgumentForward() const;
	virtual std::string getArgumentBackward() const;
	virtual std::string getType();

protected:
	dsl::debug::Debug deb;

};

} /* namespace dsl */

#endif /* COMMANDEXECUTABLE_HPP_ */
