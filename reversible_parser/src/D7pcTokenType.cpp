/*
 * D7pcTokenType.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <D7pcTokenType.h>
#include <lexer/TokenType.h>

namespace edsl {

const lexer::TokenType D7pcTokenType::TT_SEQUENCE("TT_SEQUENCE", "\\Asequence", true);
const lexer::TokenType D7pcTokenType::TT_FRAMES("TT_FRAMES","\\Aframes", true);
const lexer::TokenType D7pcTokenType::TT_JOINTQ("TT_JOINTQ","\\Ajointq", true);
const lexer::TokenType D7pcTokenType::TT_END("TT_END","\\Aend", true);

const lexer::TokenType D7pcTokenType::TT_LOCAL("TT_LOCAL", "\\Alocal",true);
const lexer::TokenType D7pcTokenType::TT_DELOCAL("TT_DELOCAL", "\\Adelocal", true);
const lexer::TokenType D7pcTokenType::TT_CALL("TT_CALL","\\Acall", true);
const lexer::TokenType D7pcTokenType::TT_UNCALL("TT_UNCALL","\\Auncall", true);

const lexer::TokenType D7pcTokenType::TT_LEFT("TT_LEFT","\\A\\(", true);
const lexer::TokenType D7pcTokenType::TT_RIGHT("TT_RIGHT","\\A\\)", true);
const lexer::TokenType D7pcTokenType::TT_DOT("TT_DOT","\\A\\.", true);
const lexer::TokenType D7pcTokenType::TT_COMMA("TT_DOT","\\A\\,", true);
const lexer::TokenType D7pcTokenType::TT_EQUAL("TT_EQUAL","\\A=.", true);
const lexer::TokenType D7pcTokenType::TT_COLON("TT_COLON","\\A:", true);
const lexer::TokenType D7pcTokenType::TT_SEMICOLON("TT_SEMICOLON","\\A;", false);
const lexer::TokenType D7pcTokenType::TT_IDENTIFIER("TT_IDENTIFIER","\\A(\\w)+", true);
const lexer::TokenType D7pcTokenType::TT_STRING("TT_STRING","\\A\"(.)*?\"", true);
//const lexer::TokenType D7pcTokenType::TT_STRING("TT_STRING","(?<=\\A\")(.)*?(?=\")", true);				//Should remove " but somehow does not

const lexer::TokenType D7pcTokenType::TT_COMMENT_LINE("TT_COMMENT_LINE","\\A//(.)*?(?=\\v)", false);				// *? is nongreede repeat - ?= is a backtrace with 0 width https://stackoverflow.com/questions/7124778/how-to-match-anything-up-until-this-sequence-of-characters-in-a-regular-expres
const lexer::TokenType D7pcTokenType::TT_WHITESPACE("TT_WHITESPACE","\\A(\\h)+", false);					// For whitespace including EOL use \s https://stackoverflow.com/questions/3469080/match-whitespace-but-not-newlines
const lexer::TokenType D7pcTokenType::TT_END_OF_LINE("TT_END_OF_LINE","\\A\\v", false);						// Newlines \v could also be detected with (\\r\\n|\\n)
const lexer::TokenType D7pcTokenType::TT_END_OF_FILE("TT_END_OF_FILE","\\AEOF", false);

std::vector<lexer::TokenType> D7pcTokenType::getTokenList()
{
	std::vector<lexer::TokenType> tokentypes;
	tokentypes.push_back(D7pcTokenType::TT_SEQUENCE);
	tokentypes.push_back(D7pcTokenType::TT_FRAMES);
	tokentypes.push_back(D7pcTokenType::TT_JOINTQ);
	tokentypes.push_back(D7pcTokenType::TT_END);

	tokentypes.push_back(D7pcTokenType::TT_LOCAL);
	tokentypes.push_back(D7pcTokenType::TT_DELOCAL);
	tokentypes.push_back(D7pcTokenType::TT_CALL);
	tokentypes.push_back(D7pcTokenType::TT_UNCALL);

	tokentypes.push_back(D7pcTokenType::TT_LEFT);
	tokentypes.push_back(D7pcTokenType::TT_RIGHT);
	tokentypes.push_back(D7pcTokenType::TT_DOT);
	tokentypes.push_back(D7pcTokenType::TT_COMMA);
	tokentypes.push_back(D7pcTokenType::TT_EQUAL);
	tokentypes.push_back(D7pcTokenType::TT_COLON);
	tokentypes.push_back(D7pcTokenType::TT_SEMICOLON);
	tokentypes.push_back(D7pcTokenType::TT_IDENTIFIER);
	tokentypes.push_back(D7pcTokenType::TT_STRING);

	tokentypes.push_back(D7pcTokenType::TT_COMMENT_LINE);
	tokentypes.push_back(D7pcTokenType::TT_WHITESPACE);
	tokentypes.push_back(D7pcTokenType::TT_END_OF_LINE);
	tokentypes.push_back(D7pcTokenType::TT_END_OF_FILE);
	return tokentypes;
}

} /* namespace edsl */
