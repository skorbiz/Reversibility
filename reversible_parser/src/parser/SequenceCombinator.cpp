/*
 * SequenceCombinator.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <lexer/Token.h>
#include <lexer/TokenBuffer.h>
#include <lexer/TokenType.h>
#include <parser/Error23.h>
#include <parser/SequenceCombinator.h>
#include <iostream>
#include <string>

namespace edsl {
namespace parser {

const std::string SequenceCombinator::matchTypeDefault("SEQUENCE_COMBINATOR");


SequenceCombinator::SequenceCombinator(std::shared_ptr<Combinator> production1) :
		matchType(matchTypeDefault)
{
	productions.push_back(production1);
}

SequenceCombinator::SequenceCombinator(std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2) :
				matchType(matchTypeDefault)
{
	productions.push_back(production1);
	productions.push_back(production2);
}

SequenceCombinator::SequenceCombinator(std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3) :
				matchType(matchTypeDefault)
{
	productions.push_back(production1);
	productions.push_back(production2);
	productions.push_back(production3);
}

SequenceCombinator::SequenceCombinator(std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3, std::shared_ptr<Combinator> production4) :
				matchType(matchTypeDefault)
{
	productions.push_back(production1);
	productions.push_back(production2);
	productions.push_back(production3);
	productions.push_back(production4);
}

SequenceCombinator::SequenceCombinator(std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3, std::shared_ptr<Combinator> production4, std::shared_ptr<Combinator> production5) :
				matchType(matchTypeDefault)
{
	productions.push_back(production1);
	productions.push_back(production2);
	productions.push_back(production3);
	productions.push_back(production4);
	productions.push_back(production5);

}

SequenceCombinator::SequenceCombinator(std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3, std::shared_ptr<Combinator> production4, std::shared_ptr<Combinator> production5, std::shared_ptr<Combinator> production6) :
				matchType(matchTypeDefault)
{
	productions.push_back(production1);
	productions.push_back(production2);
	productions.push_back(production3);
	productions.push_back(production4);
	productions.push_back(production5);
	productions.push_back(production6);

}

SequenceCombinator::SequenceCombinator(std::string matchType, std::shared_ptr<Combinator> production1) :
		matchType(matchType)
{
	productions.push_back(production1);
}

SequenceCombinator::SequenceCombinator(std::string matchType, std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2) :
				matchType(matchType)
{
	productions.push_back(production1);
	productions.push_back(production2);
}

SequenceCombinator::SequenceCombinator(std::string matchType, std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3) :
				matchType(matchType)
{
	productions.push_back(production1);
	productions.push_back(production2);
	productions.push_back(production3);
}

SequenceCombinator::SequenceCombinator(std::string matchType, std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3, std::shared_ptr<Combinator> production4) :
				matchType(matchType)
{
	productions.push_back(production1);
	productions.push_back(production2);
	productions.push_back(production3);
	productions.push_back(production4);
}

SequenceCombinator::SequenceCombinator(std::string matchType, std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3, std::shared_ptr<Combinator> production4, std::shared_ptr<Combinator> production5) :
				matchType(matchType)
{
	productions.push_back(production1);
	productions.push_back(production2);
	productions.push_back(production3);
	productions.push_back(production4);
	productions.push_back(production5);

}

SequenceCombinator::SequenceCombinator(std::string matchType, std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3, std::shared_ptr<Combinator> production4, std::shared_ptr<Combinator> production5, std::shared_ptr<Combinator> production6) :
				matchType(matchType)
{
	productions.push_back(production1);
	productions.push_back(production2);
	productions.push_back(production3);
	productions.push_back(production4);
	productions.push_back(production5);
	productions.push_back(production6);
}

SequenceCombinator::~SequenceCombinator()
{
}

ParseResult SequenceCombinator::recognizer(ParseResult inbound) const
{
	if(!inbound.isOk())
		return inbound;

	ParseResult latestsResult = inbound;
	std::vector<ParseResult> results;

	for(auto const production : productions )
	{
		if(production != nullptr)
		{
			latestsResult = production->recognizer(latestsResult);
			if(latestsResult.isOk())
				results.push_back(latestsResult);
		}
		else													//TODO incorporate right-recursion more clearly into class interface // Or maybe express it in new class using base combinators?
		{
			latestsResult = this->recognizer(latestsResult);
			if(latestsResult.isOk())
				results.push_back(latestsResult);
			else if(results.size() > 0)
				return results[0];

		}
	}

	if(results.size() == productions.size())
	{
		latestsResult = ParseResult(true, latestsResult.getTokenBuffer(), matchType);
		action(latestsResult, results);
	}
	else
	{
		auto errorResult = ParseResult(false, latestsResult.getTokenBuffer(), matchType);
		errorResult.setError(std::make_shared<Error23>("Illegal syntax with SequenceCombinator for token " + inbound.getTokenBuffer().peakNextToken().type.type, -1, latestsResult.getError()));
		latestsResult = errorResult;
	}
	return latestsResult;

}


} /* namespace parser */
} /* namespace edsl */
