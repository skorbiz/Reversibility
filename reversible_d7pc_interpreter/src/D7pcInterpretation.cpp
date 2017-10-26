/*
 * D7pcInterpretation.cpp
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#include <reversible_d7pc_model/ModelFactory.h>
#include <reversible_parser/ast/ASTNode.h>
#include <D7pcInterpretation.h>
#include <reversible_parser/D7pcTokenType.h>
#include <reversible_parser/lexer/TokenType.h>
#include <stddef.h>
#include <cassert>
#include <iostream>
#include <vector>

namespace model {
class Instruction;
} /* namespace model */

namespace edsl {

using namespace lexer;
using namespace abstract_syntax_tree;
using namespace model;


D7pcInterpretation::D7pcInterpretation(std::shared_ptr<ModelFactory> factory) : factory(factory)
{
}

D7pcInterpretation::~D7pcInterpretation()
{
}

D7pcInterpretation::Build_return_type D7pcInterpretation::interprete(std::shared_ptr<abstract_syntax_tree::ASTNode> root)
{

	auto main_sequence = root->getSoleChild(D7pcTokenType::TT_SEQUENCE.type);
	return build(main_sequence);



}


D7pcInterpretation::Build_return_type D7pcInterpretation::build(std::shared_ptr<abstract_syntax_tree::ASTNode> node)
{
	if(node->type == D7pcTokenType::TT_SEQUENCE.type)
		return build_sequence(node->getChild(0)->getContent(), node->getChild(1));
	return build_terminal(node);
}

D7pcInterpretation::Build_return_type D7pcInterpretation::build_sequence(std::string name, std::shared_ptr<abstract_syntax_tree::ASTNode> node)
{
	Build_return_type r;
	return r;
}

D7pcInterpretation::Build_return_type D7pcInterpretation::build_terminal(std::shared_ptr<abstract_syntax_tree::ASTNode> node)
{
//	if(node->getContent(0) == "print")
//		if(node->size() == 4)
//			return {createPrint(toVariable(n->getChild(2)))};
//
//	if(node->getContent(0) == "prepare_feeder")
//		return {createPrepareFeeder(toVariable(n->getChild(2)))};
//
//	if(node->getContent(0) == "restart_point")
//		return {createRestartPoint()};
//
//	if(node->getContent(0) == "nothing")
//		return {createNothing()};
//
//	if(node->getType(0) == D7pcTokenType::TT_LOCAL.type)
//		if(n->size() == 5)
//			return{
//			createDeclaration(n->getContent(1), n->getContent(2)),
//					createAssignment(n->getContent(2), toString(n->getChild(4)->getSoleChild()))};
//
//	if(n->getType(0) == D7pcTokenType::TT_LOCAL.type)
//		if(n->size() == 3)
//			return {createDeclaration(n->getContent(1), n->getContent(2))};
//
//	if(n->getType(0) == D7pcTokenType::TT_IDENTIFIER.type)
//		if(n->getType(1) == D7pcTokenType::TT_COLON.type)
//			if(n->getType(2) == D7pcTokenType::TT_EQUAL.type)
//				if(n->size() == 4)
//					return {createUpdate(n->getContent(0), n->getContent(3))};
//
//	if(n->getType(0) == D7pcTokenType::TT_IDENTIFIER.type)
//		if(n->getType(1) == D7pcTokenType::TT_EQUAL.type)
//			if(n->size() == 3)
//				return {createAssignment(n->getContent(0), toString(n->getChild(2)->getSoleChild()))};
//
//	std::cout << "Warning: D7pcInterpretation::toExecutable failed to create executable from node type " << n->getType() << " with content: " <<  n->getContent() << std::endl;
//	for(size_t i = 0; i < n->size(); i++)
//		std::cout << "  Child " << i << " had type: " << n->getType(i) << " with content: " << n->getContent(0) << std::endl;
//	std::cout << "  " << variables->varString << std::endl;
//	return {createNothing()};

	Build_return_type r;
	return r;
}






/*std::shared_ptr<model::Instruction> D7pcInterpretation::interprete(std::shared_ptr<abstract_syntax_tree::ASTNode> root)
{

	std::cout << "START" << std::endl;

	//Find main sequence
	auto sequences = root->getDecendents(D7pcTokenType::TT_SEQUENCE.type);

    struct IntermidiateSequence
    {
		std::string name;
		std::vector<std::shared_ptr<abstract_syntax_tree::ASTNode> > instructions;
    };
    std::vector<IntermidiateSequence> intermediates;

	std::cout << "FOR" << std::endl;
	for(auto s : sequences)
	{
		assert(s != nullptr);
		assert(s->abstractSyntaxTreeChilds.size() == 2);
		auto instructionList = s->abstractSyntaxTreeChilds[1];
		IntermidiateSequence is;
		is.name = s->abstractSyntaxTreeChilds[0]->getContent();
		is.instructions = instructionList->abstractSyntaxTreeChilds;
		assert(is.instructions.size() > 0);
		intermediates.push_back(is);
	}

	std::cout << "PRINT" << std::endl;
	for(auto s : intermediates)
		std::cout << "Sequence: " << s.name << " has " << s.instructions.size() << " instructions" << std::endl;



	IntermidiateSequence main;
	for(auto s: intermediates)
		if(s.name == "main")
		{
			main = s;
			break;
		}
	assert(main.name == "main");


	std::vector<std::shared_ptr<model::Instruction> >instructions;


	for(auto i : main.instructions)
	{
		auto newins = createInstruction(i);
		instructions.insert(instructions.end(), newins.begin(), newins.end());
	}

	for(unsigned int i = 0; i < instructions.size()-1; i++)
	{
		instructions[i]->setInstructionNext(instructions[i+1]);
		instructions[i+1]->setInstructionPrevious(instructions[i]);
	}


	return instructions.front();

}

std::vector<std::shared_ptr<model::Instruction> > D7pcInterpretation::createInstruction(std::shared_ptr<abstract_syntax_tree::ASTNode> n)
{
	std::vector<std::shared_ptr<model::Instruction> >instructions;
//	auto executables = toExecutable(n);
//
//	for(auto e : executables)
//	{
//		model::Identification23 id("t1",2,3,4);
//		bool isReversible = false;
//		bool isSymmetric = false;
//		bool isSwapped = false;
//		bool isUncallLayout = false;
//		std::shared_ptr<model::InstructionExecutable> executable = e;
//		e->setVariableContainer(variables);
//		std::shared_ptr<model::Instruction> instruction = std::make_shared<model::Instruction>(id, isReversible, isSymmetric, isSwapped, isUncallLayout);
//		instruction->setInstructionExecutable(executable);
//		instructions.push_back(instruction);
//	}
	return instructions;
}

std::string D7pcInterpretation::toString(std::shared_ptr<abstract_syntax_tree::ASTNode> n)
{
//	std::string str = n->getContent();
//	if(n->getType() == D7pcTokenType::TT_STRING.type)
//		return str.substr(1, str.size()-2);				//Removes the " from the string
//	std::cout << "Warning: D7pcInterpretation::toString got node type " << n->getType() << "But expected type: " << D7pcTokenType::TT_STRING.type << std::endl;
	return "ERROR";

}

int str_var_count = 0;
std::string D7pcInterpretation::toVariable(std::shared_ptr<abstract_syntax_tree::ASTNode> n)
{
//	assert(n->getType() == "ARG");
//
//	if(n->getType(0) == D7pcTokenType::TT_STRING.type)
//	{
//		std::string content = toString(n->getSoleChild());
//		std::string varName = "temp_str_var_" + std::to_string(str_var_count);
//		str_var_count += 1;
//		Variable<std::string> var(varName, content);
//		variables->varString.save(varName, var);
//		return varName;
//	}
//
//	if(n->getType(0) == D7pcTokenType::TT_IDENTIFIER.type)
//		return n->getContent(0);
//
//	std::cout << "Warning: D7pcInterpretation::toVariable got node type " << n->getType() << std::endl;
//	return "ERROR";
}








//Variable<std::string> D7pcInterpretation::toVariable(std::shared_ptr<abstract_syntax_tree::ASTNode> n)
//{
//	if(n->getType() == D7pcTokenType::TT_IDENTIFIER.type)
//		return fetchVariable(n->getContent());
//	if(n->getType() == D7pcTokenType::TT_STRING.type)
//		return model::Variable<std::string>("static_var", toString(n));
//
//	std::cout << "Warning: D7pcInterpretation::toVariable got node type " << n->getType() << std::endl;
//	return model::Variable<std::string>("ERROR", "ERROR");
//}
//
//Variable<std::string> D7pcInterpretation::fetchVariable(std::string varName)
//{
//	return variables->varString.getRef(varName);
//}
//
//Variable<std::string> D7pcInterpretation::createVariable(std::string varName)
//{
//	model::Variable<std::string> var(varName, "");
//	variables->varString.save(varName, var);
//	return var;
//}
//
//Variable<rw::kinematics::State> D7pcInterpretation::toVariableState(std::shared_ptr<abstract_syntax_tree::ASTNode> n)
//{
//	std::cout << "Warning: D7pcInterpretation::toVariable got node type " << n->getType() << std::endl;
//	return model::Variable<rw::kinematics::State>("ERROR", wc->getDefaultState());
//}
//
//Variable<rw::kinematics::State> D7pcInterpretation::createVariableState(std::string varName)
//{
//	model::Variable<rw::kinematics::State> var(varName, wc->getDefaultState());
//	variables->varState.save(varName, var);
//	return var;
//}*/



} /* namespace edsl */
