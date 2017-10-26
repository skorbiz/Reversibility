/*
 * IntermediateModel.hpp
 *
 *  Created on: Feb 13, 2015
 *      Author: josl
 */

#ifndef INTERMEDIATEMODEL_HPP_
#define INTERMEDIATEMODEL_HPP_

#include <../src/model/basemodel/Instruction.hpp>
#include <../src/model/expandedmodel/MemoryModel.hpp>

namespace dsl
{

class IntermediateModel
{

public:
	IntermediateModel();
	virtual ~IntermediateModel();
	virtual void buildBaseModel(MemoryModel<dsl::IntermediateModel*> & memory) = 0;
	virtual std::shared_ptr<dsl::Instruction> getInstructionFirst(MemoryModel<dsl::IntermediateModel*> & memory) = 0;
	virtual std::shared_ptr<dsl::Instruction> getInstructionLast(MemoryModel<dsl::IntermediateModel*> & memory) = 0;
};

} /* namespace dsl */

#endif /* INTERMEDIATEMODEL_HPP_ */
