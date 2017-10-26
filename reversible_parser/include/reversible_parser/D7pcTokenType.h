/*
 * D7pcTokenType.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PCTOKENTYPE_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PCTOKENTYPE_H_

#include <reversible_parser/lexer/TokenType.h>
#include <vector>

namespace edsl { namespace lexer { class TokenType; } /* namespace lexer */ } /* namespace edsl */

namespace edsl {

class D7pcTokenType {
public:
	static const lexer::TokenType TT_FRAMES;
	static const lexer::TokenType TT_JOINTQ;
	static const lexer::TokenType TT_SEQUENCE;
	static const lexer::TokenType TT_END;

	static const lexer::TokenType TT_LOCAL;
	static const lexer::TokenType TT_DELOCAL;
	static const lexer::TokenType TT_CALL;
	static const lexer::TokenType TT_UNCALL;

	static const lexer::TokenType TT_LEFT;
	static const lexer::TokenType TT_RIGHT;
	static const lexer::TokenType TT_DOT;
	static const lexer::TokenType TT_COMMA;
	static const lexer::TokenType TT_EQUAL;
	static const lexer::TokenType TT_COLON;
	static const lexer::TokenType TT_SEMICOLON;
	static const lexer::TokenType TT_IDENTIFIER;
	static const lexer::TokenType TT_STRING;

	static const lexer::TokenType TT_COMMENT_LINE;
	static const lexer::TokenType TT_WHITESPACE;
	static const lexer::TokenType TT_END_OF_LINE;
	static const lexer::TokenType TT_END_OF_FILE;

	static std::vector<lexer::TokenType> getTokenList();

};

} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PCTOKENTYPE_H_ */
