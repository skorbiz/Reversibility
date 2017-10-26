/*
 * RegexLexer.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_LEXER_REGEXLEXER_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_LEXER_REGEXLEXER_H_

#include <reversible_parser/lexer/Token.h>
#include <reversible_parser/lexer/TokenType.h>
#include <string>
#include <vector>

namespace edsl {
namespace lexer {

class RegexLexer {
public:
	RegexLexer();
	virtual ~RegexLexer();

	void consumeFile(std::string input, std::vector<TokenType> types);
	void to_screen();

private:
	bool matchToken();

public:
	std::vector<Token> tokenStream;
	std::string scanBuffer;
	std::vector<TokenType> tokenTypes;
};

} /* namespace lexer */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_LEXER_REGEXLEXER_H_ */


