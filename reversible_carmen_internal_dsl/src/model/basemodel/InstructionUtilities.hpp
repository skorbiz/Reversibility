/*
 * InstructionUtilities.hpp
 *
 *  Created on: Feb 17, 2015
 *      Author: josl
 */

#ifndef INSTRUCTIONUTILITIES_HPP_
#define INSTRUCTIONUTILITIES_HPP_

#include <vector>
#include <../src/model/basemodel/controller/MessagesContainer.hpp>
#include <../src/model/basemodel/Instruction.hpp>
#include <../src/model/basemodel/InstructionGraph.hpp>


namespace dsl {
namespace basemodel {

	std::vector<std::shared_ptr<dsl::Instruction> > getListOfAllInstruction(std::shared_ptr<dsl::Instruction> root);
	std::vector<std::shared_ptr<dsl::Instruction> > getListOfAllInstruction(std::shared_ptr<dsl::Instruction> in, std::vector<std::shared_ptr<dsl::Instruction> > & vec);

} /* namespace basemodel */
} /* namespace dsl */

#endif /* INSTRUCTIONUTILITIES_HPP_ */
