/*
 * Command.hpp
 *
 *  Created on: Nov 7, 2014
 *      Author: josl
 */

#ifndef COMMAND_HPP_
#define COMMAND_HPP_

#include <../src/model/basemodel/command/CommandExecutable.hpp>
#include <../src/model/basemodel/Instruction.hpp>

namespace dsl
{

class Command : public Instruction
{

public:
	Command(dsl::Identification id, std::shared_ptr<dsl::CommandExecutable> cmdExcutable, bool isReversible, bool isSymmetric, bool isSwapped, bool isUncallLayout);
	virtual ~Command();

	void doExecuteForward();
	void doExecuteBackward();

	std::string getType();

private:
	void stateUpdate(dsl::State & state);
	void stateUpdateSwapped(dsl::State & state);
	std::string getArgumentForward() const;
	std::string getArgumentBackward() const;

	std::shared_ptr<CommandExecutable> _executableCommand; //	dsl::CommandExecutable * _executableCommand;

};



// ******************************************************************
// **** TEMPLATE IMPLEMENTATIONS ************************************
// ******************************************************************

} /* namespace dsl */

#endif /* COMMAND_HPP_ */
