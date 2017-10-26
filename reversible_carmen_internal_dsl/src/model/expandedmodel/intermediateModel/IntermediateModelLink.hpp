/*
 * IntermediateModelLink.hpp
 *
 *  Created on: Feb 15, 2015
 *      Author: josl
 */

#ifndef INTERMEDIATEMODELLINK_HPP_
#define INTERMEDIATEMODELLINK_HPP_

#include <../src/model/basemodel/Identification.hpp>
#include <../src/model/expandedmodel/intermediateModel/IntermediateModel.hpp>

namespace dsl
{

class IntermediateModelLink : public IntermediateModel
{

public:
	IntermediateModelLink(dsl::Identification::Ptr idOuter, dsl::Identification::Ptr idInnerFirst, dsl::Identification::Ptr idInnerLast);
	virtual ~IntermediateModelLink();

	void buildBaseModel(MemoryModel<dsl::IntermediateModel*> & memory);
	std::shared_ptr<dsl::Instruction> getInstructionFirst(MemoryModel<dsl::IntermediateModel*> & memory);
	std::shared_ptr<dsl::Instruction> getInstructionLast(MemoryModel<dsl::IntermediateModel*> & memory);


private:
	dsl::Identification::Ptr _idOuter;
	dsl::Identification::Ptr _idInnerFirst;
	dsl::Identification::Ptr _idInnerLast;
};

} /* namespace dsl */

#endif /* INTERMEDIATEMODELLINK_HPP_ */
