/*
 * Sequence.cpp
 *
 *  Created on: Nov 17, 2014
 *      Author: josl
 */

#include "Sequence.hpp"

namespace dsl {

Sequence::Sequence(std::string name) :
		_name(name),
		_instance(0),
		_depth(-2),
		_start(nullptr),
		_end(nullptr)
{
}

Sequence::~Sequence()
{
}


// **********************************
// *** Build functions
void Sequence::addElment(std::shared_ptr<Element> element)
{
//	gen::dExtendConstruction << this << " :: ElementSequence :: addElment" << std::endl;
	_elements.push_back(element);
}

void Sequence::build()
{
	MemoryModel<dsl::IntermediateModel *> intermediateModel;
	buildIntermediateModel(direction::FORWARD, false, true, createName("mainHnd"), nullptr, nullptr, intermediateModel);
	IntermediateModel * start = intermediateModel.getObject(_name + " [0.start]");
	start->buildBaseModel(intermediateModel);
}

std::shared_ptr<dsl::Instruction> Sequence::getInstructionStart()
{
	return _start;
}

std::shared_ptr<dsl::Instruction> Sequence::getInstructionEnd()
{
	return _end;
}



void Sequence::buildIntermediateModel(direction dir, bool isUncallLayout, bool isReversible, dsl::Identification::Ptr id, dsl::Identification::Ptr next, dsl::Identification::Ptr previous, MemoryModel<dsl::IntermediateModel*> & memory)
{
	_depth = id->getDepth();

	buildConditionChecks();
	buildHandles();
	buildAppendedArguments();
	buildElements(dir, isUncallLayout, isReversible, next, previous, memory);
	buildReverseCounterpart(dir, isUncallLayout, memory);
	buildIntermediateLink(dir, id, memory);

	//Gets reverse and end instructions
	_start = memory.getObject(createName("start")->toString())->getInstructionFirst(memory);
	_end = memory.getObject(createName("end")->toString())->getInstructionFirst(memory);

	_instance++;
}


// **********************************
// *** Build functions

void Sequence::buildConditionChecks()
{
	if(_elements.size() < 1)
	{
		std::cerr << "Sequence had size: "<< _elements.size() << std::endl;
		std::cerr << "Sequences of size < 1 can't be build" << std::endl;
		exit(-1);
	}

	std::shared_ptr<dsl::Element> reverseSequence = getReverseCounterpart();
	std::shared_ptr<dsl::Element> reverseElement = _elements[_elements.size()-1]->getReverseCounterpart();
	if(reverseSequence != nullptr)
		if(reverseElement != nullptr)
			std::cerr << "WARNING: Reverse of last element in " << _name << ""
					" was overwritten be sequence reverse" << std::endl;
}


void Sequence::buildAppendedArguments()
{
//	for(unsigned int e = 0; e < _elements.size(); e++)
//		for(unsigned int p = 0; p < _permanentArguments.size(); p++)
//			_elements[e]->appendTemporaryArgument(_permanentArguments[p]);
//
//	for(unsigned int e = 0; e < _elements.size(); e++)
//		for(unsigned int t = 0; t < _temporaryArguments.size(); t++)
//			_elements[e]->appendTemporaryArgument(_temporaryArguments[t]);
//
//	_temporaryArguments.clear();
}

void Sequence::buildHandles()
{
	dsl::Identification::Ptr nameStart = createName("start");
	dsl::Identification::Ptr nameEnd = createName("end");

	if(_instance == 0)
	{
		std::shared_ptr<dsl::EmptyLinker> efStart = std::make_shared<EmptyLinker>(nameStart->toString());
		std::shared_ptr<dsl::EmptyLinker> efEnd = std::make_shared<EmptyLinker>(nameEnd->toString());

		std::shared_ptr<ElementCommand> hndStart = std::make_shared<dsl::ElementCommand>(efStart);
		std::shared_ptr<ElementCommand> hndEnd = std::make_shared<dsl::ElementCommand>(efEnd);

		_elements.insert(_elements.begin(), hndStart);
		_elements.push_back(hndEnd);
	}
}

void Sequence::buildElements(direction dir, bool isUncallLayout, bool isReversible, dsl::Identification::Ptr next, dsl::Identification::Ptr previous, MemoryModel<dsl::IntermediateModel*> & memory)
{
	dsl::Identification::Ptr nameStart = createName("start");
	dsl::Identification::Ptr nameEnd = createName("end");

	std::vector<dsl::Identification::Ptr> ids;
	std::vector<dsl::Identification::Ptr> nxt;
	std::vector<dsl::Identification::Ptr> bck;

	ids.push_back(nameStart);
	for(unsigned int e = 1; e < _elements.size() -1; e++)
		ids.push_back(createName(e));
	ids.push_back(nameEnd);

	for(unsigned int e = 1; e < _elements.size() -1; e++)
		nxt.push_back(createName(e));
	nxt.push_back(nameEnd);
	nxt.push_back(next);

	bck.push_back(previous);
	bck.push_back(nameStart);
	for(unsigned int e = 1; e < _elements.size() -1; e++)
		bck.push_back(createName(e));

	//Handles reverse elements
	for(unsigned int e = 1; e < _elements.size() -1; e++)
		if(_elements[e]->getReverseCounterpart() != nullptr)
			bck[e+1] = createName("r" + e);

	//Handles reverse of sequence
	if(getReverseCounterpart() != nullptr)
		bck[bck.size()-1] = createName("r");

	//Handles start and end pointer if reverse
	if(dir == direction::REVERSE){
		bck[0] = next;
		nxt[nxt.size()-1] = previous;
	}

	//Swaps forward and reverse handles in nessesary
	if(dir == direction::REVERSE)
		nxt.swap(bck);

	//Create local isReversible variable
	bool isReversibleLocal = isReversible and _isReversible;

	//Builds elements
	for(unsigned int e = 0; e < _elements.size(); e++)
			_elements[e]->buildIntermediateModel(dir, isUncallLayout, isReversibleLocal, ids[e], nxt[e], bck[e], memory);

	//Builds reverse elements
	for(unsigned int e = 1; e < _elements.size()-1; e++)
	{
		bool isUncallLayoutTemp = false;
		std::shared_ptr<dsl::Element> reverse = _elements[e]->getReverseCounterpart();
		if(reverse != nullptr)
			reverse->buildIntermediateModel(dir, isUncallLayoutTemp, true, createName("r" + e), nxt[e], bck[e], memory);
	}
}



void Sequence::buildReverseCounterpart(direction dir, bool isUncallLayout, MemoryModel<dsl::IntermediateModel*> & memory)
{
	dsl::Identification::Ptr nameStart = createName("start");
	dsl::Identification::Ptr nameEnd = createName("end");

	std::shared_ptr<dsl::Element> reverse = getReverseCounterpart();
	if(dir == direction::FORWARD && reverse != nullptr)
		reverse->buildIntermediateModel(dir, isUncallLayout, true, createName("r"), nameEnd, nameStart, memory);
	else if( dir == direction::REVERSE && reverse != nullptr)
		reverse->buildIntermediateModel(dir, isUncallLayout, true, createName("r"), nameStart, nameEnd, memory);
}

void Sequence::buildIntermediateLink(direction dir, dsl::Identification::Ptr nameOriginal, MemoryModel<dsl::IntermediateModel*> & memory)
{

	IntermediateModelLink * iml;
	if(dir == direction::FORWARD)
		iml = new IntermediateModelLink(nameOriginal, createName("start"), createName("end"));
	else
		iml = new IntermediateModelLink(nameOriginal, createName("end"), createName("start"));
	memory.saveObject(nameOriginal->toString(),iml);

}

dsl::Identification::Ptr Sequence::createName(std::string id)
{
	return dsl::Identification::Ptr(new dsl::Identification(_name, _instance, id, _depth+1));
}

dsl::Identification::Ptr Sequence::createName(int id)
{
	return dsl::Identification::Ptr(new dsl::Identification(_name, _instance, id, _depth+1));
}



} /* namespace dsl */
