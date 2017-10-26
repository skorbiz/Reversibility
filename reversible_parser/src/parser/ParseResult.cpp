/*
 * ParseResult.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <ast/ASTNode.h>
#include <lexer/Token.h>
#include <parser/ParseResult.h>
#include <iostream>

namespace edsl {
namespace parser {


ParseResult::ParseResult(bool grammarOk, lexer::TokenBuffer tokenbuffer, std::string matchType) :
	grammarOk(grammarOk),
	matchType(matchType),
	matchToken(nullptr),
	tokens(tokenbuffer)
{
}

ParseResult::ParseResult(bool grammarOk, lexer::TokenBuffer tokenbuffer, lexer::Token matchToken, std::string matchType) :
	grammarOk(grammarOk),
	matchType(matchType),
	matchToken(std::make_shared<lexer::Token>(matchToken)),
	tokens(tokenbuffer)
{
}

ParseResult::ParseResult(bool grammarOk, lexer::TokenBuffer tokenbuffer, std::shared_ptr<lexer::Token> matchToken, std::string matchType) :
	grammarOk(grammarOk),
	matchType(matchType),
	matchToken(matchToken),
	tokens(tokenbuffer)
{
}

ParseResult::~ParseResult()
{
}

bool ParseResult::isOk()
{
	return grammarOk;
}

std::string ParseResult::getMatchType()
{
	return matchType;
}

std::shared_ptr<lexer::Token> ParseResult::getMatchToken()
{
	return matchToken;
}

lexer::TokenBuffer ParseResult::getTokenBuffer()
{
	return tokens;
}

std::shared_ptr<abstract_syntax_tree::ASTNode> ParseResult::getActionResult()
{
	return actionResult;
}

void ParseResult::setActionResult(std::shared_ptr<abstract_syntax_tree::ASTNode> actResult)
{
	actionResult = actResult;
}

void ParseResult::to_screen()
{
	std::cout << "Combinator result match: " << grammarOk << std::endl;
	tokens.to_screen();
}

std::shared_ptr<Error23> ParseResult::getError()
{
	return error;
}

void ParseResult::setError(std::shared_ptr<Error23> e)
{
	error = e;
}


} /* namespace parser */
} /* namespace edsl */
