/*
 * Terminal.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <lexer/Token.h>
#include <lexer/TokenBuffer.h>
#include <parser/Error23.h>
#include <parser/Terminal.h>
#include <string>

namespace edsl {
namespace parser {

Terminal::Terminal(lexer::TokenType match) :
				matchType(match.type),
				tokenMatch(match)
{
}

Terminal::Terminal(lexer::TokenType match, std::string matchType) :
				matchType(matchType),
				tokenMatch(match)
{
}

Terminal::~Terminal()
{
}

ParseResult Terminal::recognizer(ParseResult inbound) const
{
	if(!inbound.isOk())
		return inbound;

	ParseResult result = inbound;
	lexer::TokenBuffer tokenBuffer = inbound.getTokenBuffer();
	lexer::Token t = tokenBuffer.peakNextToken();

	if(t.isOfType(tokenMatch))
	{
		lexer::TokenBuffer tokenBufferOutTokens = tokenBuffer.makePoppedTokkenList();
		result = ParseResult(true,tokenBufferOutTokens, t, matchType);
		action(result);
//		std::cout << "Suc " << t.type.type << " | " << tokenMatch.type << std::endl;

	}
	else
	{
		result = ParseResult(false,tokenBuffer, t, matchType);
        result.setError(std::make_shared<Error23>("Illegal syntax: [" + t.payload + "]. Expected " + tokenMatch.type + " but found " + t.type.type, -1));
//		std::cout << "fail " << t.type.type << " | " << tokenMatch.type << std::endl;

	}
	return result;
}

} /* namespace parser */
} /* namespace edsl */
