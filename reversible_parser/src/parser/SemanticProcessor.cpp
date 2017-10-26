/*
 * SemanticProcessor.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <parser/SemanticProcessor.h>
#include <iostream>

namespace edsl {
namespace parser {

SemanticProcessor::SemanticProcessor()
{
}

SemanticProcessor::~SemanticProcessor()
{
}

void SemanticProcessor::handleParseResult(std::vector<ParseResult> & steps, ParseResult & result) const
{
	std::cout <<__PRETTY_FUNCTION__ << std::endl;
}


} /* namespace parser */
} /* namespace edsl */
