/*
 * GraphRepresentation.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: josl
 */

#include "GraphRepresentation.hpp"

namespace dsl {
namespace common {

GraphRepresentation::GraphRepresentation(dsl::BaseProgram aAssemblyProgram) :
		_maxRank(-1)
{
	deb.setPrefix("[GraphRepresentation] ");
	deb.setColorPrefix(dsl::color::CYAN);
//	deb.disable();
	buildGraph(aAssemblyProgram.getStart());
}

GraphRepresentation::~GraphRepresentation()
{
}

int GraphRepresentation::getMaxRank()
{
	return _maxRank;
}

std::vector<std::shared_ptr<dsl::Instruction> > GraphRepresentation::getInstructions()
{
	std::vector<std::shared_ptr<dsl::Instruction> > ret;
	for ( std::map<std::shared_ptr<dsl::Instruction>, Node>::iterator it = _nodes.begin(); it != _nodes.end(); ++it)
		ret.push_back(it->second.instruction);
	return ret;
}


std::vector<std::shared_ptr<dsl::Instruction> > GraphRepresentation::getInstructionOfRank(int rank)
{
	std::vector<std::shared_ptr<dsl::Instruction> > ret;
	for ( std::map<std::shared_ptr<dsl::Instruction>, Node>::iterator it = _nodes.begin(); it != _nodes.end(); ++it)
		if(it->second.rank == rank )
			ret.push_back(it->second.instruction);
	return ret;
}

//std::vector<dsl::Instruction *> GraphRepresentation::getEdges(std::function<bool(bool, bool)> comparison)
//{
//	std::vector<dsl::Instruction *> ret;
//	for ( map<dsl::Instruction *, Node>::iterator it = _nodes.begin(); it != _nodes.end(); ++it)
//		if(comparison(it->second.forward , it->second.backward))
//		{
//			ret.push_back(it->second.from);
//			ret.push_back(it->second.to);
//		}
//	return ret;
//}
//
//std::vector<dsl::Instruction *> GraphRepresentation::getForwardExlusivEdges()
//{
//	  return getEdges([](bool x, bool y){ return y == false; } );
//}


std::vector<std::shared_ptr<dsl::Instruction> > GraphRepresentation::getOmniDirectionalEdges()
{
	std::vector<std::shared_ptr<dsl::Instruction> > ret;
	for ( std::map<std::shared_ptr<dsl::Instruction >, Node>::iterator it = _nodes.begin(); it != _nodes.end(); ++it)
	{
		std::shared_ptr<dsl::Instruction> current = it->second.instruction;
		std::shared_ptr<dsl::Instruction> next = current->getInstructionNext();

		if(next == nullptr)
			continue;

		std::shared_ptr<dsl::Instruction> isCurrent = next->getInstructionPrevious();

		if( current == isCurrent )
		{
			ret.push_back(current);
			ret.push_back(next);
		}
	}
	deb << "size: " << ret.size() << std::endl;
	return ret;
}

std::vector<std::shared_ptr<dsl::Instruction> > GraphRepresentation::getForwardExlusivEdges()
{
	std::vector<std::shared_ptr<dsl::Instruction> > ret;
	for ( std::map<std::shared_ptr<dsl::Instruction>, Node>::iterator it = _nodes.begin(); it != _nodes.end(); ++it)
	{
		std::shared_ptr<dsl::Instruction> current = it->second.instruction;
		std::shared_ptr<dsl::Instruction> next = current->getInstructionNext();

		if(next == nullptr)
			continue;

		std::shared_ptr<dsl::Instruction> isCurrent = next->getInstructionPrevious();

		if( current != isCurrent )
		{
			ret.push_back(current);
			ret.push_back(next);
		}
	}
	return ret;

}

std::vector<std::shared_ptr<dsl::Instruction> > GraphRepresentation::getBackwardExclusivEdges()
{
	std::vector<std::shared_ptr<dsl::Instruction> > ret;
	for ( std::map<std::shared_ptr<dsl::Instruction>, Node>::iterator it = _nodes.begin(); it != _nodes.end(); ++it)
	{
		std::shared_ptr<dsl::Instruction> current = it->second.instruction;
		std::shared_ptr<dsl::Instruction> prev = current->getInstructionPrevious();

		if(prev == nullptr)
			continue;

		std::shared_ptr<dsl::Instruction> isCurrent = prev->getInstructionNext();

		if( current != isCurrent )
		{
			ret.push_back(current);
			ret.push_back(prev);
		}
	}
	return ret;
}



void GraphRepresentation::buildGraph(std::shared_ptr<dsl::Instruction> ins)
{
	deb << "Start graph building" << std::endl;
	buildNodes(ins);
	deb << "Build Nodes - number of nodes: "<< _nodes.size() << std::endl;
	buildEdges(ins);
	deb << "Build Edges" << std::endl;
	clearVisited();
	deb << "Clear visited" << std::endl;
	rankGraph(ins, 0);
	deb << "Rank graph - max rank: " << _maxRank << std::endl;
	deb << "End graph building" << std::endl;
}

void GraphRepresentation::buildNodes(std::shared_ptr<dsl::Instruction> ins)
{
	if(ins == nullptr)
		return;

	if(_nodes.count(ins) != 0)
		return;

	Node n;
	n.instruction = ins;
	n.visited = false;
	n.rank = -1;
	_nodes[ins] = n;

	buildNodes(ins->getInstructionNext());
	buildNodes(ins->getInstructionPrevious());
}

void GraphRepresentation::buildEdges(std::shared_ptr<dsl::Instruction> ins)
{
	if(ins == nullptr)
		return;

	if(_nodes[ins].visited == true)
		return;
	_nodes[ins].visited = true;

	std::shared_ptr<dsl::Instruction> next = ins->getInstructionNext();
	std::shared_ptr<dsl::Instruction> prev = ins->getInstructionPrevious();

	if(next != nullptr)		_nodes[ins].topDownEdges.push_back(next);
	if(prev != nullptr)		_nodes[prev].topDownEdges.push_back(ins);

	buildEdges(next);
	buildEdges(prev);
}

void GraphRepresentation::clearVisited()
{
	for ( std::map<std::shared_ptr<dsl::Instruction>, Node>::iterator it = _nodes.begin(); it != _nodes.end(); ++it)
		it->second.visited = false;
}

void GraphRepresentation::rankGraph(std::shared_ptr<dsl::Instruction> ins, int rank)
{
	if(ins == nullptr)
		return;

	if(_nodes[ins].rank > 10000)
		std::cerr << "error in graph print: node: " << _nodes[ins].instruction->getID() << " had rank: " << _nodes[ins].rank << std::endl;

	if(_nodes[ins].rank >= rank)
		return;

	if(_maxRank < rank)
		_maxRank = rank;

	_nodes[ins].rank = rank;

	for(unsigned int i = 0; i < _nodes[ins].topDownEdges.size(); i++)
		rankGraph(_nodes[ins].topDownEdges[i], rank+1);
}

//bool GraphRepresentation::isOmniConnected(dsl::Instruction * from, dsl::Instruction * to)
//{
//	assert(to != nullptr);
//	assert(from != nullptr);
//
//	if(from->getInstructionNext() == to && to->getInstructionPrevious() == from)
//		return true;
//
//	if(from->getInstructionPrevious() == to && to->getInstructionNext() == from)
//		return true;
//
//	return false;
//}




} /* namespace common */
} /* namespace dsl */
