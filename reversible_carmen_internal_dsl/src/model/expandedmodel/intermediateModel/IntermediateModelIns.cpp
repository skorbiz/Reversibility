/*
 * IntermediateModelIns.cpp
 *
 *  Created on: Feb 15, 2015
 *      Author: josl
 */

#include "IntermediateModelIns.hpp"

namespace dsl {

IntermediateModelIns::IntermediateModelIns(std::shared_ptr<dsl::Instruction> ins, dsl::Identification::Ptr id, dsl::Identification::Ptr next, dsl::Identification::Ptr previous) :
		_ins(ins),
		_idThis(id),
		_idNext(next),
		_idPrevious(previous),
		build(false)
{
}

IntermediateModelIns::~IntermediateModelIns()
{
}

void IntermediateModelIns::buildBaseModel(MemoryModel<dsl::IntermediateModel*> & memory)
{
	if(build == true)
		return;
	build = true;

	if(_idNext != nullptr)
	{
		dsl::IntermediateModel * next = memory.getObject(_idNext->toString());
		_ins->setInstructionNext(next->getInstructionFirst(memory));
		next->buildBaseModel(memory);
	}

	if(_idPrevious != nullptr)
	{
		dsl::IntermediateModel * prev = memory.getObject(_idPrevious->toString());
		_ins->setInstructionPrevious(prev->getInstructionLast(memory));
		prev->buildBaseModel(memory);
	}
}

std::shared_ptr<dsl::Instruction> IntermediateModelIns::getInstructionFirst(MemoryModel<dsl::IntermediateModel*> & memory)
{
	return _ins;
}

std::shared_ptr<dsl::Instruction> IntermediateModelIns::getInstructionLast(MemoryModel<dsl::IntermediateModel*> & memory)
{
	return _ins;
}


} /* namespace dsl */
