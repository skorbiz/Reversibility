/*
 * OrCombinator.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <assert.h>
#include <lexer/Token.h>
#include <lexer/TokenBuffer.h>
#include <lexer/TokenType.h>
#include <parser/Error23.h>
#include <parser/OrCombinator.h>
#include <iostream>
#include <string>

namespace edsl {
namespace parser {

OrCombinator::OrCombinator(std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2) :
		matchType("OR_COMBINATOR")
{
	productions.push_back(production1);
	productions.push_back(production2);
}

OrCombinator::OrCombinator(std::string matchType, std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2) :
				matchType(matchType)
{
	productions.push_back(production1);
	productions.push_back(production2);
}

OrCombinator::OrCombinator(std::initializer_list<std::shared_ptr<Combinator> >input) :
		matchType("OR_COMBINATOR")
{
	for(auto i : input)
		productions.push_back(i);
}

OrCombinator::OrCombinator(std::string matchType, std::initializer_list<std::shared_ptr<Combinator> >input) :
				matchType(matchType)
{
	for(auto i : input)
		productions.push_back(i);
}



OrCombinator::~OrCombinator()
{
}

ParseResult OrCombinator::recognizer(ParseResult inbound) const
{
	if(!inbound.isOk())
		return inbound;

	ParseResult latestsResult = inbound;

	for(auto const production : productions )
	{
		latestsResult = production->recognizer(inbound);
		if(latestsResult.isOk())
		{
			auto result = ParseResult(true, latestsResult.getTokenBuffer(), matchType);
			std::vector<ParseResult> results = {latestsResult};
			action(result, results);
			return result;
		}
	}

	auto errorResult = ParseResult(false, latestsResult.getTokenBuffer(), matchType);
	errorResult.setError(std::make_shared<Error23>("Illegal syntax with OrCombinator for token " + inbound.getTokenBuffer().peakNextToken().type.type, -1, latestsResult.getError()));
	latestsResult = errorResult;

	return latestsResult;
}


} /* namespace parser */
} /* namespace edsl */
