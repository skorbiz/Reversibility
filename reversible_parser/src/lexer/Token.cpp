/*
 * Token.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <lexer/Token.h>

namespace edsl {
namespace lexer {

Token::Token(TokenType type, std::string payload) :
	type(type),
	payload(payload)
{
}

Token::~Token()
{
}

bool Token::isOfType(const TokenType & input) const
{
	return type.isEqual(input);
}

std::ostream& operator<<(std::ostream& os, const Token& t)
{
    os << "Token{"<< t.type.type << "," << t.payload << "}";
    return os;
}


} /* namespace lexer */
} /* namespace edsl */
