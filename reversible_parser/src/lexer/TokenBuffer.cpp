/*
 * TokenBuffer.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <assert.h>
#include <lexer/TokenBuffer.h>
#include <algorithm>
#include <iostream>

namespace edsl {
namespace lexer {

TokenBuffer::TokenBuffer(std::vector<Token> buffer, bool flipOrder)
{
	if(flipOrder)
		std::reverse(buffer.begin(),buffer.end());
	tokenbuffer = buffer;
}

TokenBuffer::~TokenBuffer()
{
}


Token TokenBuffer::peakNextToken()
{
	if(tokenbuffer.size() == 0)
	{
		TokenType type("udef","",false);
		Token token(type,"udef");
		return token;
	}

	return tokenbuffer.back();
}

TokenBuffer TokenBuffer::makePoppedTokkenList()
{
	std::vector<Token> poppedBuffer(tokenbuffer.begin(), tokenbuffer.end()-1);
	return TokenBuffer(poppedBuffer);
}

size_t TokenBuffer::size()
{
	return tokenbuffer.size();
}

void TokenBuffer::to_screen()
{
	std::cout << "Tokenbuffer size: " << tokenbuffer.size() << std::endl;
	for(auto const & t : tokenbuffer)
		std::cout << " " << t << std::endl;
}


} /* namespace lexer */
} /* namespace edsl */
