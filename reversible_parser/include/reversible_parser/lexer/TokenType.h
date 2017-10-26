/*
 * TokenType.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_LEXER_TOKENTYPE_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_LEXER_TOKENTYPE_H_

#include <string>

namespace edsl {
namespace lexer {

class TokenType
{

public:
	TokenType(std::string type, std::string regExPattern, bool outputToken);
	virtual ~TokenType();

	bool isEqual(const TokenType & tokentype) const;

	std::string type;
	std::string regExPattern;
	bool outputToken;

//	typedef std::set<const TokenType*> instances_list;
//	typedef instances_list::const_iterator const_iterator;
//	static instances_list token_type_instances;
//	static const_iterator begin(void) { return token_type_instances.begin(); }
//	static const_iterator end(void) { return token_type_instances.end(); }
};

} /* namespace lexer */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_LEXER_TOKENTYPE_H_ */
