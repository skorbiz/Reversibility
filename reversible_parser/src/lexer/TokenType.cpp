/*
 * TokenType.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <lexer/TokenType.h>

namespace edsl {
namespace lexer {

//std::set< const TokenType*> TokenType::token_type_instances;

TokenType::TokenType(std::string type, std::string regExPattern, bool outputToken) :
	type(type),
	regExPattern(regExPattern),
	outputToken(outputToken)
{
//	token_type_instances.insert(this);
}

TokenType::~TokenType()
{
}


bool TokenType::isEqual(const TokenType & input) const
{
	if(type == input.type)
		if(regExPattern == input.regExPattern)
			if(outputToken == input.outputToken)
				return true;
	return false;
}

} /* namespace lexer */
} /* namespace edsl */
