/*
 * Combinator.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <ast/ASTNode.h>
#include <ast/ASTProcessor.h>
#include <parser/Combinator.h>
#include <iostream>

namespace edsl {
namespace parser {

Combinator::Combinator() :
	processor(std::make_shared<abstract_syntax_tree::ASTProcessor>())
{
}

Combinator::~Combinator()
{
}

std::shared_ptr<Combinator> Combinator::withSemanticProccessor(std::shared_ptr<SemanticProcessor> aProcessor)
{
	processor = aProcessor;
	return shared_from_this();
}

void Combinator::action(ParseResult & result, std::vector<ParseResult> steps) const
{
	processor->handleParseResult(steps, result);
}

} /* namespace parser */
} /* namespace edsl */
