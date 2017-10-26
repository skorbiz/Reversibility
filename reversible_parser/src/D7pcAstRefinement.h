/*
 * D7pcAstRefinement.h
 *
 *  Created on: Oct 5, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_PARSER_SRC_D7PCASTREFINEMENT_H_
#define D7PC_ROS_REVERSIBLE_PARSER_SRC_D7PCASTREFINEMENT_H_

#include <memory>
#include <string>

#include "../include/reversible_parser/parser/ParseResult.h"
#include "../include/reversible_parser/ast/ASTNode.h"

namespace edsl {

class D7pcAstRefinement
{
public:
	D7pcAstRefinement();
	virtual ~D7pcAstRefinement();

	parser::ParseResult transform_ast(parser::ParseResult input);


protected:
	std::shared_ptr<abstract_syntax_tree::ASTNode> create_string_node(std::string identifier, std::string value);


};

} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_PARSER_SRC_D7PCASTREFINEMENT_H_ */
