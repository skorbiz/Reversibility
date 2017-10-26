/*
 * InstructionGraph.cpp
 *
 *  Created on: May 29, 2015
 *      Author: josl
 */

#include "InstructionGraph.hpp"

namespace dsl {

InstructionGraph::InstructionGraph(std::shared_ptr<dsl::Instruction> ins) :
		_insStart(nullptr),
		_insEnd(nullptr)
{
	buildGraph(ins);
}

InstructionGraph::~InstructionGraph()
{
}

std::shared_ptr<dsl::Instruction> InstructionGraph::getStartInstruction()
{
	assert( _insStart != nullptr );
	return _insStart;
}

std::shared_ptr<dsl::Instruction> InstructionGraph::getEndInstruction()
{
	assert( _insEnd != nullptr );
	return _insEnd;
}

std::vector<std::shared_ptr<dsl::Instruction> > InstructionGraph::getAll()
{
	std::vector<std::shared_ptr<dsl::Instruction> > allIns;
	for(auto imap: _nodes)
	    allIns.push_back(imap.first);
	return allIns;
}

std::vector<std::shared_ptr<dsl::Instruction> > InstructionGraph::getConnectedToForward(std::shared_ptr<dsl::Instruction> ins)
{
	assert(ins != nullptr);
	return _nodes[ins].forwardConncetionTo;
}

std::vector<std::shared_ptr<dsl::Instruction> > InstructionGraph::getConnectedToBackward(std::shared_ptr<dsl::Instruction> ins)
{
	assert(ins != nullptr);
	return _nodes[ins].backwardsConnectionTo;
}

void InstructionGraph::buildGraph(std::shared_ptr<dsl::Instruction> ins)
{
	if(ins == nullptr)
		return;

	if(_nodes.count(ins) != 0)
		return;

	Node n;
	n.visited = false;
	_nodes[ins] = n;

	std::shared_ptr<dsl::Instruction> next = ins->getInstructionNext();
	std::shared_ptr<dsl::Instruction> prev = ins->getInstructionPrevious();

	buildGraph(next);
	buildGraph(prev);

	if(next != nullptr)
		_nodes[next].forwardConncetionTo.push_back(ins);

	if(prev != nullptr)
		_nodes[prev].backwardsConnectionTo.push_back(ins);

	if(next == nullptr)
	{
		assert(_insEnd == nullptr );
		_insEnd = ins;
	}

	if(prev == nullptr)
	{
		assert(_insStart == nullptr );
		_insStart = ins;
	}
}

void InstructionGraph::clearVisited()
{
}



} /* namespace dsl */
