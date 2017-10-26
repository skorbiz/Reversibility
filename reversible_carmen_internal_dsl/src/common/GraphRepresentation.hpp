/*
 * GraphRepresentation.hpp
 *
 *  Created on: Mar 11, 2015
 *      Author: josl
 */

#ifndef GRAPHREPRESENTATION_HPP_
#define GRAPHREPRESENTATION_HPP_

#include <functional>
#include <iostream>
#include <random>
#include <../src/model/basemodel/BaseProgram.hpp>
#include <../src/model/basemodel/Instruction.hpp>
#include <debug/Debug.hpp>
#include <color/colors.hpp>

namespace dsl {
namespace common {

class GraphRepresentation
{

	struct Node
	{
		std::shared_ptr<dsl::Instruction> instruction;
		std::vector<std::shared_ptr<dsl::Instruction> > topDownEdges;
		int rank;
		bool visited;
	};

public:
	GraphRepresentation(dsl::BaseProgram aAssemblyProgram);
	virtual ~GraphRepresentation();

	int getMaxRank();
	std::vector<std::shared_ptr<dsl::Instruction> > getInstructions();
	std::vector<std::shared_ptr<dsl::Instruction> > getInstructionOfRank(int rank);
	std::vector<std::shared_ptr<dsl::Instruction> > getOmniDirectionalEdges();
	std::vector<std::shared_ptr<dsl::Instruction> > getForwardExlusivEdges();
	std::vector<std::shared_ptr<dsl::Instruction> > getBackwardExclusivEdges();

private:
	void buildGraph(std::shared_ptr<dsl::Instruction> ins);
	void buildNodes(std::shared_ptr<dsl::Instruction> ins);
	void buildEdges(std::shared_ptr<dsl::Instruction> ins);
	void clearVisited();
	void rankGraph(std::shared_ptr<dsl::Instruction> ins, int rank);

private:
	std::map<std::shared_ptr<dsl::Instruction>, Node> _nodes;
	int _maxRank;
	dsl::debug::Debug deb;

};

} /* namespace common */
} /* namespace dsl */

#endif /* GRAPHREPRESENTATION_HPP_ */


