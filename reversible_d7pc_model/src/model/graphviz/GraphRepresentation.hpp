/*
 * GraphRepresentation.hpp
 *
 *  Created on: Mar 11, 2015
 *      Author: josl
 */

#ifndef GRAPHREPRESENTATION_HPP_
#define GRAPHREPRESENTATION_HPP_

#include <debug/Debug.hpp>
#include <map>
#include <memory>
#include <vector>

namespace model {
class Instruction;
} /* namespace model */

namespace dsl {
namespace common {

class GraphRepresentation
{

	struct Node
	{
		std::shared_ptr<model::Instruction> instruction;
		std::vector<std::shared_ptr<model::Instruction> > topDownEdges;
		int rank;
		bool visited;
	};

public:
	GraphRepresentation(std::shared_ptr<model::Instruction>);
	virtual ~GraphRepresentation();

	int getMaxRank();
	std::vector<std::shared_ptr<model::Instruction> > getInstructions();
	std::vector<std::shared_ptr<model::Instruction> > getInstructionOfRank(int rank);
	std::vector<std::shared_ptr<model::Instruction> > getOmniDirectionalEdges();
	std::vector<std::shared_ptr<model::Instruction> > getForwardExlusivEdges();
	std::vector<std::shared_ptr<model::Instruction> > getBackwardExclusivEdges();

private:
	void buildGraph(std::shared_ptr<model::Instruction> ins);
	void buildNodes(std::shared_ptr<model::Instruction> ins);
	void buildEdges(std::shared_ptr<model::Instruction> ins);
	void clearVisited();
	void rankGraph(std::shared_ptr<model::Instruction> ins, int rank);

private:
	std::map<std::shared_ptr<model::Instruction>, Node> _nodes;
	int _maxRank;
	dsl::debug::Debug deb;

};

} /* namespace common */
} /* namespace dsl */

#endif /* GRAPHREPRESENTATION_HPP_ */


