/*
 * Terminal.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_TERMINAL_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_TERMINAL_H_

#include <lexer/TokenType.h>
#include <parser/Combinator.h>

namespace edsl {
namespace parser {
class ParseResult;
} /* namespace parser */
} /* namespace edsl */

namespace edsl {
namespace parser {

class Terminal : public Combinator
{

public:
	Terminal(lexer::TokenType match);
	Terminal(lexer::TokenType match, std::string matchType);
	virtual ~Terminal();

	ParseResult recognizer(ParseResult inbound) const;

private:
	std::string matchType;
	lexer::TokenType tokenMatch;

};

} /* namespace parser */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_TERMINAL_H_ */
