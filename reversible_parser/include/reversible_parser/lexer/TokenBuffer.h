/*
 * TokenBuffer.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_LEXER_TOKENBUFFER_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_LEXER_TOKENBUFFER_H_

#include <reversible_parser/lexer/Token.h>
#include <vector>

namespace edsl {
namespace lexer {

class TokenBuffer {
public:
public:
	TokenBuffer();
	TokenBuffer(std::vector<Token> buffer, bool flipOrder = false);
	virtual ~TokenBuffer();

	Token peakNextToken();
	TokenBuffer makePoppedTokkenList();

	size_t size();
	void to_screen();

private:
	std::vector<Token> tokenbuffer;
};

} /* namespace lexer */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_LEXER_TOKENBUFFER_H_ */
