/*
 * RegexLexer.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <boost/regex/v4/regex.hpp>
#include <boost/regex/v4/regex_fwd.hpp>
#include <boost/regex/v4/regex_search.hpp>
#include <lexer/RegexLexer.h>
#include <cassert>
#include <iomanip>
#include <iostream>

namespace edsl {
namespace lexer {

RegexLexer::RegexLexer()
{
}

RegexLexer::~RegexLexer()
{
}

void RegexLexer::consumeFile(std::string input, std::vector<TokenType> type)
{
	scanBuffer = input;
	tokenTypes = type;

	bool parseInProgress = true;
	while(parseInProgress)
	{
		parseInProgress = matchToken();
	}

	//Errors
	if(type.size() == 0)			std::cout << "Warning: RegexLexer got empty list of tokenTypes to recognize" << std::endl;
	if(tokenStream.size() == 0) 	std::cout << "Warning: RegexLexer failed to recognize any Tokens" << std::endl;
	if(scanBuffer.size() > 0)		std::cout << "Warning: RegexLexer has unrecognized scanbuffer => " + scanBuffer << std::endl;
}

bool RegexLexer::matchToken()
{


	for(auto const& token_type : tokenTypes)
	{
		boost::regex expr(token_type.regExPattern);
		boost::smatch what;

		bool is_token_matched = boost::regex_search(scanBuffer, what, expr);

//		std::cout << token_type.regExPattern << std::endl;
//		std::cout << scanBuffer.substr(0, 10) << std::endl;
//		std::cout << std::endl;


		if(is_token_matched)
		{
			std::string payload = what[0];

//			std::cout << "matched" << std::endl;

			if(token_type.outputToken)
			{
				Token t(token_type, what[0]);
				tokenStream.push_back(t);
			}

			scanBuffer = scanBuffer.substr(payload.length());
			return true;
		}
	}

	return false;
}

void RegexLexer::to_screen()
{
	std::cout << "RegexLexer tokenStream : " << std::endl;
	for(unsigned int i = 0; i < tokenStream.size(); i++)
		std::cout
		<< std::left << std::setw(20) << tokenStream[i].type.type
		<< std::left << std::setw(20) << tokenStream[i].payload
		<< std::left << std::setw(15) << tokenStream[i].type.regExPattern
		<< std::endl;
}

} /* namespace lexer */
} /* namespace edsl */





