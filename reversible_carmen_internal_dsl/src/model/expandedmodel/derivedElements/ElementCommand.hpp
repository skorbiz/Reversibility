/*
 * ElementCommand.hpp
 *
 *  Created on: Dec 9, 2014
 *      Author: josl
 */

#ifndef ELEMENTCOMMAND_HPP_
#define ELEMENTCOMMAND_HPP_

#include <cassert>
#include <iostream>
#include <../src/model/basemodel/command/Command.hpp>
#include <../src/model/basemodel/command/CommandExecutable.hpp>
#include <../src/model/basemodel/Instruction.hpp>
#include <../src/model/expandedmodel/intermediateModel/IntermediateModelIns.hpp>
#include <../src/model/expandedmodel/Element.hpp>

namespace dsl
{

class AssemblyProgram;

class ElementCommand : public dsl::Element
{

public:
	ElementCommand(std::shared_ptr<dsl::CommandExecutable> cmdExecutable);
	virtual ~ElementCommand();
	void buildIntermediateModel(direction dir, bool uncallLayout, bool isReversible, dsl::Identification::Ptr id, dsl::Identification::Ptr next, dsl::Identification::Ptr previous, MemoryModel<dsl::IntermediateModel * > & memory);
private:
	virtual std::shared_ptr<dsl::Command> createCommand(dsl::Identification::Ptr id, std::shared_ptr<dsl::CommandExecutable> cmdExecutable, bool switchedDirection, bool isUncallLayout, bool isReversible);

private:
	std::shared_ptr<dsl::CommandExecutable> _cmd;

};

} /* namespace dsl */

#endif /* ELEMENTCOMMAND_HPP_ */
