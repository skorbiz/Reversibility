/*
 * Sequence.hpp
 *
 *  Created on: Nov 17, 2014
 *      Author: josl
 */

#ifndef SEQUENCE_HPP_
#define SEQUENCE_HPP_

#include <vector>
#include <string>

#include <../src/model/basemodel/argument/Argument.hpp>
#include <../src/model/basemodel/command/derivedCommands/EmptyLinker.hpp>
#include <../src/model/basemodel/Identification.hpp>
#include <../src/model/expandedmodel/Element.hpp>
#include <../src/model/expandedmodel/derivedElements/ElementCommand.hpp>
#include <../src/model/expandedmodel/intermediateModel/IntermediateModelLink.hpp>
#include <../src/model/expandedmodel/MemoryModel.hpp>
namespace dsl
{

class Sequence : public Element
{

public:
	Sequence(std::string name);
	virtual ~Sequence();

	//Building functions
	void addElment(std::shared_ptr<dsl::Element> element);
	void build();
	std::shared_ptr<dsl::Instruction> getInstructionStart();
	std::shared_ptr<dsl::Instruction> getInstructionEnd();

	//Overwritten functions
	void buildIntermediateModel(direction dir, bool isUncallLayout, bool isReversible, dsl::Identification::Ptr id, dsl::Identification::Ptr next, dsl::Identification::Ptr previous, MemoryModel<dsl::IntermediateModel*> & memory);

private:
	//Build functions
	void buildConditionChecks();
	void buildAppendedArguments();
	void buildHandles();
	void buildElements(direction dir, bool isUncallLayout, bool isReversible, dsl::Identification::Ptr next, dsl::Identification::Ptr previous, MemoryModel<dsl::IntermediateModel*> & memory);
	void buildReverseCounterpart(direction dir, bool isUncallLayout, MemoryModel<dsl::IntermediateModel*> & memory);
	void buildIntermediateLink( direction dir, dsl::Identification::Ptr nameOriginal, MemoryModel<dsl::IntermediateModel*> & memory );

	dsl::Identification::Ptr createName(std::string id);
	dsl::Identification::Ptr createName(int id);

private:
	std::string _name;
	int _instance;
	int _depth;
	std::vector<std::shared_ptr<dsl::Element> > _elements;
	std::shared_ptr<dsl::Instruction> _start;
	std::shared_ptr<dsl::Instruction> _end;
};

} /* namespace dsl */

#endif /* SEQUENCE_HPP_ */
