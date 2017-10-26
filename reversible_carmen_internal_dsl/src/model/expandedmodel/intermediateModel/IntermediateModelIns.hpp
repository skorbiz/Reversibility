/*
 * IntermediateModelIns.hpp
 *
 *  Created on: Feb 15, 2015
 *      Author: josl
 */

#ifndef INTERMEDIATEMODELINS_HPP_
#define INTERMEDIATEMODELINS_HPP_

#include <../src/model/basemodel/Instruction.hpp>
#include <../src/model/expandedmodel/intermediateModel/IntermediateModel.hpp>

namespace dsl
{

class IntermediateModelIns : public IntermediateModel
{
public:
	IntermediateModelIns(std::shared_ptr<dsl::Instruction> ins, dsl::Identification::Ptr id, dsl::Identification::Ptr next, dsl::Identification::Ptr previous);
	virtual ~IntermediateModelIns();

	void buildBaseModel(MemoryModel<dsl::IntermediateModel*> & memory);
	std::shared_ptr<dsl::Instruction> getInstructionFirst(MemoryModel<dsl::IntermediateModel*> & memory);
	std::shared_ptr<dsl::Instruction> getInstructionLast(MemoryModel<dsl::IntermediateModel*> & memory);

private:
	std::shared_ptr<dsl::Instruction> _ins;
	dsl::Identification::Ptr _idThis;
	dsl::Identification::Ptr _idNext;
	dsl::Identification::Ptr _idPrevious;
	bool build;


};

} /* namespace dsl */

#endif /* INTERMEDIATEMODELINS_HPP_ */
