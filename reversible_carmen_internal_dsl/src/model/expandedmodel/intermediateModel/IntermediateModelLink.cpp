/*
 * IntermediateModelLink.cpp
 *
 *  Created on: Feb 15, 2015
 *      Author: josl
 */

#include "IntermediateModelLink.hpp"

namespace dsl {

IntermediateModelLink::IntermediateModelLink(dsl::Identification::Ptr idOuter, dsl::Identification::Ptr idInnerFirst, dsl::Identification::Ptr idInnerLast) :
		_idOuter(idOuter),
		_idInnerFirst(idInnerFirst),
		_idInnerLast(idInnerLast)
{
}

IntermediateModelLink::~IntermediateModelLink()
{
}

void IntermediateModelLink::buildBaseModel(MemoryModel<dsl::IntermediateModel*> & memory)
{
	IntermediateModel * inner = memory.getObject(_idInnerFirst->toString());
	inner->buildBaseModel(memory);
}

std::shared_ptr<dsl::Instruction> IntermediateModelLink::getInstructionFirst(MemoryModel<dsl::IntermediateModel*> & memory)
{
	IntermediateModel * inner = memory.getObject(_idInnerFirst->toString());
	return inner->getInstructionFirst(memory);
}

std::shared_ptr<dsl::Instruction> IntermediateModelLink::getInstructionLast(MemoryModel<dsl::IntermediateModel*> & memory)
{
	IntermediateModel * inner = memory.getObject(_idInnerLast->toString());
	return inner->getInstructionLast(memory);

}

} /* namespace dsl */
