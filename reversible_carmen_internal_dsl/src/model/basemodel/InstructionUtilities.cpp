/*
 * InstructionUtilities.cpp
 *
 *  Created on: Feb 17, 2015
 *      Author: josl
 */

#include <vector>
#include "InstructionUtilities.hpp"

namespace dsl {
namespace basemodel {

	std::vector<std::shared_ptr<dsl::Instruction> > getListOfAllInstruction(std::shared_ptr<dsl::Instruction> root)
	{
		std::vector<std::shared_ptr<dsl::Instruction> > vec;
		return getListOfAllInstruction(root, vec);
	}

	std::vector<std::shared_ptr<dsl::Instruction> > getListOfAllInstruction(std::shared_ptr<dsl::Instruction> ins, std::vector<std::shared_ptr<dsl::Instruction> > & vec)
	{
		if(ins == nullptr)
			return vec;

		for(unsigned int i = 0; i < vec.size(); i++)
			if(ins == vec[i])
				return vec;

		vec.push_back(ins);

		std::shared_ptr<dsl::Instruction> next = ins->getInstructionNext();
		std::shared_ptr<dsl::Instruction> prev = ins->getInstructionPrevious();

		vec = getListOfAllInstruction(next, vec);
		vec = getListOfAllInstruction(prev, vec);
		return vec;
	}


} /* namespace basemodel */
} /* namespace dsl */
