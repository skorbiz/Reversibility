/*
 * main.cpp
 *
 *  Created on: Jan 23, 2017
 *      Author: josl
 */

#include <ros/package.h>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "../include/reversible_parser/ast/AST2Graphviz.h"
#include "../include/reversible_parser/ast/ASTNode.h"
#include "../include/reversible_parser/D7pcParser.h"
#include "../include/reversible_parser/D7pcTokenType.h"
#include "../include/reversible_parser/lexer/RegexLexer.h"
#include "../include/reversible_parser/lexer/TokenBuffer.h"
#include "../include/reversible_parser/parser/ParseResult.h"
#include "../include/reversible_parser/support/File23.h"
#include "../include/reversible_parser/support/GraphvizPrinter.h"
#include "D7pcAstRefinement.h"


int main(int argc, char **argv)
{

	using namespace edsl;
	using namespace edsl::lexer;

	std::cout << "program start" << std::endl;

	std::cout << "### File read ###" << std::endl;
	std::string filepath = ros::package::getPath("reversible_parser") + "/src/";
	std::string filename = "edsl.txt";
	File23 file(filepath, filename);
	std::string input = file.readFile();

	std::cout << "### LEXING ###" << std::endl;
	RegexLexer lexer;
	lexer.consumeFile(input, D7pcTokenType::getTokenList());
	lexer::TokenBuffer tokenbuffer(lexer.tokenStream, true);
	lexer.to_screen();

	std::cout << "### PARSING ###" << std::endl;
	parser::ParseResult result = D7pcParser::parse(tokenbuffer);

	std::cout << "### SYNTAX TREE PRINT###" << std::endl;
	std::string filepath_dot = "/home/josl/Documents/catkin_ws/src/D7PC_ROS/reversible_parser/src/gennerated_code/ast_raw_graph.dot";
	std::string filepath_png = "/home/josl/Documents/catkin_ws/src/D7PC_ROS/reversible_parser/src/gennerated_code/ast_raw_graph.png";
	std::shared_ptr<abstract_syntax_tree::GraphvizPrinter> graphdot = std::make_shared<abstract_syntax_tree::GraphvizPrinter>(filepath_dot, filepath_png);
	abstract_syntax_tree::AST2Graphviz::print(graphdot, result.getActionResult());
	graphdot.reset();

	std::cout << "### ABSTRACT SYNTAX TREE REFINEMENT ###" << std::endl;
	D7pcAstRefinement d7pc_ast_refinement;
	parser::ParseResult ast = d7pc_ast_refinement.transform_ast(result);
	std::string filepath_ast_refined_dot = "/home/josl/Documents/catkin_ws/src/D7PC_ROS/reversible_parser/src/gennerated_code/ast_refined_graph.dot";
	std::string filepath_ast_refined_png = "/home/josl/Documents/catkin_ws/src/D7PC_ROS/reversible_parser/src/gennerated_code/ast_refined_graph.png";
	std::shared_ptr<abstract_syntax_tree::GraphvizPrinter> graphdot_2 = std::make_shared<abstract_syntax_tree::GraphvizPrinter>(filepath_ast_refined_dot, filepath_ast_refined_png);
	abstract_syntax_tree::AST2Graphviz::print(graphdot_2, ast.getActionResult());
	graphdot.reset();



/*
	std::cout << "### CODE GENNERATION ###" << std::endl;
	D7pcCodeGenneration code_genneration(result.getActionResult());

	std::cout << "### USEING CODEGEN ###" << std::endl;
//	edsl::D7pcString2Variable vars;
//	std::cout << "Use code test: " << vars.toQ("wc.qInspectOuterShell") << std::endl;

	std::cout << "### INTERPRETATION ###" << std::endl;
	D7pcInterpretation interpreter;
	auto instruction_root = interpreter.interprete(result.getActionResult());

	std::cout << "### MODEL PRINT ###" << std::endl;
	dsl::common::GraphRepresentation graph123(instruction_root);
	dsl::common::GraphPrint graphprint(graph123);

	std::cout << "### MODEL EXECUTION ###" << std::endl;
	model::ControlUnit control(instruction_root);
	control.execute();

	std::cout << "program end" << std::endl;
	return 0;
*/
}
