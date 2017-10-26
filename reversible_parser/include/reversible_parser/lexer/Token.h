/*
 * Token.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_LEXER_TOKEN_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_LEXER_TOKEN_H_

#include <reversible_parser/lexer/TokenType.h>
#include <ostream>
#include <string>

namespace edsl {
namespace lexer {

class Token
{

public:
	Token(TokenType type, std::string payload);
	virtual ~Token();

	bool isOfType(const TokenType & type) const;

    friend std::ostream& operator<<(std::ostream& os, const Token& t);

	TokenType type;
	std::string payload;

};

} /* namespace lexer */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_LEXER_TOKEN_H_ */
