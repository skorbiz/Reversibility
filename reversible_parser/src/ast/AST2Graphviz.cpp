/*
 * AST2Graphviz.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <ast/AST2Graphviz.h>
#include <ast/ASTNode.h>
#include <support/GraphvizPrinter.h>
#include <iostream>
#include <sstream>

namespace edsl {
namespace abstract_syntax_tree {

AST2Graphviz::AST2Graphviz()
{
}

AST2Graphviz::~AST2Graphviz()
{
}


void AST2Graphviz::print(std::shared_ptr<GraphvizPrinter> dotAstGraph, std::shared_ptr<ASTNode> parent)
{
	if(parent == nullptr)
		return;

	std::string idParent = id(parent);
	dotAstGraph->writeNode(idParent, GraphvizPrinter::label(parent->type + "\n" + parent->getContent()));
//	dotAstGraph->writeNode(idParent, GraphvizPrinter::label(parent->type));

	for(auto & child : parent->abstractSyntaxTreeChilds)
		if(child != nullptr)
			dotAstGraph->writeConnection(idParent, id(child));
		else
			dotAstGraph->writeConnection(idParent, "nullptr");

	for(auto & child : parent->abstractSyntaxTreeChilds)
		AST2Graphviz::print(dotAstGraph, child);

}

std::string AST2Graphviz::id(std::shared_ptr<ASTNode> node)
{
	const void * address = static_cast<const void*>(node.get());
	std::stringstream ss;
	ss << address;
	std::string id = ss.str();
	return id;
}


} /* namespace abstract_syntax_tree */
} /* namespace edsl */
