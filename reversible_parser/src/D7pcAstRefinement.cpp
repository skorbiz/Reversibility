/*
 * D7pcAstRefinement.cpp
 *
 *  Created on: Oct 5, 2017
 *      Author: josl
 */

#include "D7pcAstRefinement.h"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "../include/reversible_parser/ast/ASTNode.h"
#include "../include/reversible_parser/D7pcAstType.h"
#include "../include/reversible_parser/D7pcTokenType.h"
#include "../include/reversible_parser/lexer/TokenType.h"

using namespace edsl::abstract_syntax_tree;

namespace edsl {

D7pcAstRefinement::D7pcAstRefinement()
{
}

D7pcAstRefinement::~D7pcAstRefinement()
{
}


parser::ParseResult D7pcAstRefinement::transform_ast(parser::ParseResult input)
{
	std::cout << "test1"<< std::endl;


	std::vector<std::shared_ptr<ASTNode> > print_cmds = input.getActionResult()->getDecendentsWithContent("print");

	//REMOVES BRACKETS
	for(auto& print_cmd : print_cmds)
	{
		print_cmd->abstractSyntaxTreeParent.lock()->eraseChild(3);
		print_cmd->abstractSyntaxTreeParent.lock()->eraseChild(1);
	}

	//CHANGE ARGS TO BE DIRECTLY
	for(auto& print_cmd : print_cmds)
	{
		std::shared_ptr<ASTNode> instruction_head = print_cmd->abstractSyntaxTreeParent.lock();
		std::shared_ptr<ASTNode> instruction_list = instruction_head->abstractSyntaxTreeParent.lock();

		std::cout << instruction_head->child(1)->getSoleChild()->type << std::endl;
		if(instruction_head->child(1)->getSoleChild()->type == D7pcTokenType::TT_STRING.type)
		{

			std::string identifier = "my_random_identifier";
			std::string value = instruction_head->child(1)->getSoleChild()->content->payload;
			std::shared_ptr<ASTNode> string_node = create_string_node(identifier, value);

			instruction_head->child(1)->getSoleChild()->type = D7pcTokenType::TT_IDENTIFIER.type;
			instruction_head->child(1)->getSoleChild()->content = std::make_shared<lexer::Token>(D7pcTokenType::TT_IDENTIFIER,identifier);

			std::shared_ptr<ASTNode> list_new = std::make_shared<ASTNode>();
			list_new->type = D7pcAstType::AT_INSTRUCTION_LIST;
			list_new->abstractSyntaxTreeChilds.push_back(string_node);
			list_new->abstractSyntaxTreeChilds.push_back(instruction_head);
			string_node->abstractSyntaxTreeParent = list_new;
			instruction_head->abstractSyntaxTreeParent = list_new;

			instruction_list->replaceChild(instruction_head, list_new);

		}
	}





	return input;




//	print_cmds.erase(
//	  std::remove_if(
//			  print_cmds.begin(), print_cmds.end(),
//	    [](std::shared_ptr<ASTNode> node) { return node->content->payload != "print"; }
//	  ),
//	  print_cmds.end()
//	);
//
//	std::cout << print_cmds.size() << std::endl;

//
//	std::cout << "test" << std::endl;
//	parser::ParseResult r(true, input.getTokenBuffer(), "test");
//	return r;

}


std::shared_ptr<abstract_syntax_tree::ASTNode> D7pcAstRefinement::create_string_node(std::string identifier, std::string value)
{
	std::shared_ptr<ASTNode> local_string = std::make_shared<ASTNode>();
	std::shared_ptr<ASTNode> local_string_local = std::make_shared<ASTNode>();
	std::shared_ptr<ASTNode> local_string_identifier_type = std::make_shared<ASTNode>();
	std::shared_ptr<ASTNode> local_string_identifier_name = std::make_shared<ASTNode>();
	std::shared_ptr<ASTNode> local_string_equal = std::make_shared<ASTNode>();
	std::shared_ptr<ASTNode> local_string_arg = std::make_shared<ASTNode>();
	std::shared_ptr<ASTNode> local_string_arg_value = std::make_shared<ASTNode>();

	local_string->type = D7pcAstType::AT_INSTRUCTION;
	local_string_local->type = D7pcTokenType::TT_LOCAL.type;
	local_string_identifier_type->type = D7pcTokenType::TT_IDENTIFIER.type;
	local_string_identifier_name->type = D7pcTokenType::TT_IDENTIFIER.type;
	local_string_equal->type = D7pcTokenType::TT_EQUAL.type;
	local_string_arg->type = D7pcAstType::AT_ARGUMENT;
	local_string_arg_value->type = D7pcTokenType::TT_IDENTIFIER.type;

	local_string->abstractSyntaxTreeChilds.push_back(local_string_local);
	local_string->abstractSyntaxTreeChilds.push_back(local_string_identifier_type);
	local_string->abstractSyntaxTreeChilds.push_back(local_string_identifier_name);
	local_string->abstractSyntaxTreeChilds.push_back(local_string_equal);
	local_string->abstractSyntaxTreeChilds.push_back(local_string_arg);
	local_string_arg->abstractSyntaxTreeChilds.push_back(local_string_arg_value);

	local_string_local->abstractSyntaxTreeParent = local_string;
	local_string_identifier_type->abstractSyntaxTreeParent = local_string;
	local_string_identifier_name->abstractSyntaxTreeParent = local_string;
	local_string_equal->abstractSyntaxTreeParent = local_string;
	local_string_arg->abstractSyntaxTreeParent = local_string;
	local_string_arg_value->abstractSyntaxTreeParent = local_string_arg;

	local_string_local->content = std::make_shared<lexer::Token>(D7pcTokenType::TT_LOCAL,"local");
	local_string_identifier_type->content = std::make_shared<lexer::Token>(D7pcTokenType::TT_IDENTIFIER,"string");
	local_string_identifier_name->content = std::make_shared<lexer::Token>(D7pcTokenType::TT_IDENTIFIER,identifier);
	local_string_arg_value->content = std::make_shared<lexer::Token>(D7pcTokenType::TT_IDENTIFIER,value);

	return local_string;

}



} /* namespace edsl */
