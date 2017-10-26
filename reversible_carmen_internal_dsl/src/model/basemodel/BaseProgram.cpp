/*
 * BaseProgram.cpp
 *
 *  Created on: May 30, 2015
 *      Author: josl
 */

#include "BaseProgram.hpp"

namespace dsl{

BaseProgram::BaseProgram(std::shared_ptr<dsl::Instruction> ins) :
		_graph(ins)
{
	_deb.setPrefix("[BaseProgram] ");
	_deb << "Constructor" << std::endl;
	//	_deb.disable();
}

BaseProgram::~BaseProgram()
{
}

std::shared_ptr<dsl::Instruction> BaseProgram::getStart()
{
	return _graph.getStartInstruction();
}

std::shared_ptr<dsl::Instruction> BaseProgram::getEnd()
{
	return _graph.getEndInstruction();
}

void BaseProgram::initInstructions(dsl::MessagesContainer * container)
{
	_deb << "initInstructions" << std::endl;
	_deb << "initContainer" << std::endl;
	initContainers(container);
	_deb << "initStates" << std::endl;
	initStates();
}

void BaseProgram::initContainers(dsl::MessagesContainer * container)
{
		for(auto imap:_graph.getAll())
			imap->initMessageContainer(container);
}

void BaseProgram::initStates()
{
	dsl::State s;

	_deb << "initStates : forward" << std::endl;
	//initStatesForwardRecursive(s, _graph.getStartInstruction());
	InitStatesForwardLoop(s,_graph.getStartInstruction());

	_deb << "initStates : backwards" << std::endl;
	//initStatesReverseRecursive(s, _graph.getEndInstruction());
	InitStatesReverseLoop(s, _graph.getEndInstruction());
}

void BaseProgram::initStatesForwardRecursiveLEGACY(dsl::State state, std::shared_ptr<dsl::Instruction> ins)
{
	std::cout << "initStatesForwardRecursive is LEGACY,"
				 "replaced with InitStatesForwardLoop to handle large recursions" << std::endl;
	state = ins->initStateForward(state);
	for(auto imap: _graph.getConnectedToBackward(ins))
		initStatesForwardRecursiveLEGACY(state, imap);
}

void BaseProgram::initStatesReverseRecursiveLEGACY(dsl::State state, std::shared_ptr<dsl::Instruction> ins)
{
	std::cout << "initStatesReverseRecursive is LEGACY,"
				 "replaced with InitStatesReverseLoop to handle large recursions" << std::endl;
	state = ins->initStateReverse(state);
	for(auto imap: _graph.getConnectedToForward(ins))
		initStatesReverseRecursiveLEGACY(state, imap);
}

void BaseProgram::InitStatesForwardLoop(dsl::State state, std::shared_ptr<dsl::Instruction> ins)
{
	std::vector<dsl::State> stackS;
	std::vector<std::shared_ptr<dsl::Instruction> > stackI;

	stackS.push_back(state);
	stackI.push_back(ins);

	while(stackS.size() > 0)
	{
		dsl::State s = stackS.back();
		std::shared_ptr<dsl::Instruction> i = stackI.back();
		stackS.pop_back();
		stackI.pop_back();

		dsl::State sNew = i->initStateForward(s);
		std::vector<std::shared_ptr<dsl::Instruction> > connectedInstructions = _graph.getConnectedToBackward(i);

		for(unsigned int i = 0; i < connectedInstructions.size(); i++){
			stackS.push_back(sNew);
			stackI.push_back(connectedInstructions[i]);
		}
	}
}

void BaseProgram::InitStatesReverseLoop(dsl::State state, std::shared_ptr<dsl::Instruction> ins)
{
	std::vector<dsl::State> stackS;
	std::vector<std::shared_ptr<dsl::Instruction> > stackI;

	stackS.push_back(state);
	stackI.push_back(ins);

	while(stackS.size() > 0)
	{
		dsl::State s = stackS.back();
		std::shared_ptr<dsl::Instruction> i = stackI.back();
		stackS.pop_back();
		stackI.pop_back();

		dsl::State sNew = i->initStateReverse(s);
		std::vector<std::shared_ptr<dsl::Instruction> > connectedInstructions = _graph.getConnectedToForward(i);

		for(unsigned int i = 0; i < connectedInstructions.size(); i++){
			stackS.push_back(sNew);
			stackI.push_back(connectedInstructions[i]);
		}
	}
}


} /* namespace dsl */
