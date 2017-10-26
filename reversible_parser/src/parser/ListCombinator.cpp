/*
 * ListCombinator.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <lexer/Token.h>
#include <lexer/TokenBuffer.h>
#include <lexer/TokenType.h>
#include <parser/Error23.h>
#include <parser/ListCombinator.h>
#include <iostream>
#include <string>
#include <vector>

namespace edsl {
namespace parser {

ListCombinator::ListCombinator(std::shared_ptr<Combinator> productions) :
			matchType("LIST_COMBINATOR"),
			production(productions)
{
}

ListCombinator::ListCombinator(std::string matchType, std::shared_ptr<Combinator> productions) :
			matchType(matchType),
			production(productions)
{
}


ListCombinator::~ListCombinator()
{
}

ParseResult ListCombinator::recognizer(ParseResult inbound) const
{
	if(!inbound.isOk())
		return inbound;

	ParseResult latestsResult = inbound;

	std::vector<ParseResult> results;

	while(latestsResult.isOk())
	{
		latestsResult = production->recognizer(latestsResult);
		if(latestsResult.isOk())
			results.push_back(latestsResult);
	}

	if(results.size() > 0)
	{
		auto returnResult = ParseResult(true, latestsResult.getTokenBuffer(), matchType);
		action(returnResult, results);
		latestsResult = returnResult;
	}
	else
	{
		auto errorResult = ParseResult(false, latestsResult.getTokenBuffer(), matchType);
		errorResult.setError(std::make_shared<Error23>("Illegal syntax with ListCombinator for token " + inbound.getTokenBuffer().peakNextToken().type.type, -1, latestsResult.getError()));
		latestsResult = errorResult;
	}
	return latestsResult;

}



} /* namespace parser */
} /* namespace edsl */
