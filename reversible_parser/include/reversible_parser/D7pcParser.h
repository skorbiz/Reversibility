/*
 * D7pcParser.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PCPARSER_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PCPARSER_H_

#include <reversible_parser/lexer/TokenBuffer.h>

namespace edsl {
namespace parser {
class ParseResult;
} /* namespace parser */
} /* namespace edsl */

namespace edsl {

class D7pcParser
{
public:
	D7pcParser();
	virtual ~D7pcParser();

	static parser::ParseResult parse(lexer::TokenBuffer tokenbuffer);
};

} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PCPARSER_H_ */
