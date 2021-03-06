/*
 * InstructionGraph.hpp
 *
 *  Created on: May 29, 2015
 *      Author: josl
 */

#ifndef SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_INSTRUCTIONGRAPH_HPP_
#define SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_INSTRUCTIONGRAPH_HPP_

#include <functional>
#include <iostream>
#include <cassert>
#include <map>
#include <debug/Debug.hpp>
#include <color/colors.hpp>
#include <../src/model/basemodel/Instruction.hpp>

namespace dsl
{


class InstructionGraph
{
	struct Node
	{
		std::vector<std::shared_ptr<dsl::Instruction> > backwardsConnectionTo;
		std::vector<std::shared_ptr<dsl::Instruction> > forwardConncetionTo;
		bool visited;
	};

public:
	InstructionGraph(std::shared_ptr<dsl::Instruction> ins);
	virtual ~InstructionGraph();

	std::shared_ptr<dsl::Instruction> getStartInstruction();
	std::shared_ptr<dsl::Instruction> getEndInstruction();
	std::vector<std::shared_ptr<dsl::Instruction> > getAll();
	std::vector<std::shared_ptr<dsl::Instruction> > getConnectedToForward(std::shared_ptr<dsl::Instruction> ins);
	std::vector<std::shared_ptr<dsl::Instruction> > getConnectedToBackward(std::shared_ptr<dsl::Instruction> ins);

private:
	void buildGraph(std::shared_ptr<dsl::Instruction> ins);
	void clearVisited();

private:
	std::map<std::shared_ptr<dsl::Instruction>, Node> _nodes;
	std::shared_ptr<dsl::Instruction> _insStart;
	std::shared_ptr<dsl::Instruction> _insEnd;

	dsl::debug::Debug deb;

};

} /* namespace dsl */

#endif /* SOURCE_DIRECTORY__SDU_MAGIC_SRC_DSL_MODEL_BASEMODEL_INSTRUCTIONGRAPH_HPP_ */


//
//
//

