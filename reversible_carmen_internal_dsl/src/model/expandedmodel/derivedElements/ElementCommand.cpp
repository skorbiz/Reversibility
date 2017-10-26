/*
 * ElementCommand.cpp
 *
 *  Created on: Dec 9, 2014
 *      Author: josl
 */

#include "ElementCommand.hpp"

namespace dsl {

ElementCommand::ElementCommand(std::shared_ptr<dsl::CommandExecutable> cmdExecutable) :
	_cmd(cmdExecutable)
{
}

ElementCommand::~ElementCommand()
{
}

void ElementCommand::buildIntermediateModel(direction dir, bool uncallLayout, bool isReversible, dsl::Identification::Ptr id, dsl::Identification::Ptr next, dsl::Identification::Ptr previous, MemoryModel<dsl::IntermediateModel * > & memory)
{
	bool switchedDirection = false;
	if(dir == direction::REVERSE)
		switchedDirection = true;

	bool tempIsReversible = isReversible and _isReversible;

	std::shared_ptr<dsl::Command> cmd = createCommand(id, _cmd , switchedDirection, uncallLayout, tempIsReversible);

	IntermediateModel * im = new IntermediateModelIns(cmd, id, next, previous);
	memory.saveObject(id->toString(), im);

}

std::shared_ptr<dsl::Command> ElementCommand::createCommand(dsl::Identification::Ptr id, std::shared_ptr<dsl::CommandExecutable> cmdExecutable, bool switchedDirection, bool isUncallLayout, bool isReversible)
{
	assert(cmdExecutable != nullptr);

	std::shared_ptr<dsl::Command> cmd = std::make_shared<dsl::Command>(*id, cmdExecutable, isReversible, false, switchedDirection, isUncallLayout);
	return cmd;
}



} /* namespace dsl */
