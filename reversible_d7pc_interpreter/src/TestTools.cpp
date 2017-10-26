/*
 * TestTools.cpp
 *
 *  Created on: Nov 20, 2016
 *      Author: josl
 */

// Bring in my package's API, which is what I'm testing
#include <ast/ASTProcessor.h>
#include <File23.h>
#include <gtest/gtest.h>
#include <lexer/RegexLexer.h>
#include <lexer/Token.h>
#include <lexer/TokenBuffer.h>
#include <lexer/TokenType.h>
#include <parser/ListCombinator.h>
#include <parser/ParseResult.h>
#include <parser/SequenceCombinator.h>
#include <parser/Terminal.h>
#include <ros/package.h>
#include <memory>
#include <string>
#include <vector>


TEST(Lexer, File23)
{
	std::string filepath = ros::package::getPath("reversible_demo") + "/test/";
	std::string filename =  "test_input_file_1.txt";

	File23 input(filepath, filename);
	EXPECT_FALSE(input.isEndOfFile());
	EXPECT_EQ("", input.readLine());
	EXPECT_EQ("events", input.readLine());
	EXPECT_EQ("\tdoorClosed D1CL", input.readLine());
	EXPECT_EQ("\tdrawOpened D2OP", input.readLine());
	EXPECT_EQ("", input.readLine());
	EXPECT_EQ("end", input.readLine());
	EXPECT_TRUE(input.isEndOfFile());
	EXPECT_EQ("", input.readLine());
}

TEST(Lexer, RegexLexer)
{
	std::string in_text = "events \n\t doorClosed D1CL \n\t drawOpened D2OP \n end";

	edsl::lexer::TokenType TT_EVENT("TT_EVENT","\\Aevents", true);
	edsl::lexer::TokenType TT_END("TT_END","\\Aend", true);
	edsl::lexer::TokenType TT_IDENTIFIER("TT_IDENTIFIER","\\A(\\w)+", true);
	edsl::lexer::TokenType TT_WHITESPACE("TT_WHITESPACE","\\A(\\s)+", false);

	std::vector<edsl::lexer::TokenType> tokentypes;
	tokentypes.push_back(TT_EVENT);
	tokentypes.push_back(TT_END);
	tokentypes.push_back(TT_IDENTIFIER);
	tokentypes.push_back(TT_WHITESPACE);

	edsl::lexer::RegexLexer lexer;
	lexer.consumeFile(in_text, tokentypes);

	EXPECT_EQ("TT_EVENT", lexer.tokenStream[0].type.type);
	EXPECT_EQ("TT_IDENTIFIER", lexer.tokenStream[1].type.type);
	EXPECT_EQ("TT_IDENTIFIER", lexer.tokenStream[2].type.type);
	EXPECT_EQ("TT_END", lexer.tokenStream[5].type.type);

	EXPECT_EQ("events", lexer.tokenStream[0].payload);
	EXPECT_EQ("doorClosed", lexer.tokenStream[1].payload);
	EXPECT_EQ("D1CL", lexer.tokenStream[2].payload);
}

TEST(Lexer, Parser)
{
	std::string in_text = "events \n\t doorClosed D1CL \n\t drawOpened D2OP \n end";

	edsl::lexer::TokenType TT_EVENT("TT_EVENT","\\Aevents", true);
	edsl::lexer::TokenType TT_END("TT_END","\\Aend", true);
	edsl::lexer::TokenType TT_IDENTIFIER("TT_IDENTIFIER","\\A(\\w)+", true);

	std::vector<edsl::lexer::Token> tokenstream;
	tokenstream.push_back(edsl::lexer::Token(TT_EVENT, "events"));
	tokenstream.push_back(edsl::lexer::Token(TT_IDENTIFIER, "doorClosed"));
	tokenstream.push_back(edsl::lexer::Token(TT_IDENTIFIER, "D1CL"));
	tokenstream.push_back(edsl::lexer::Token(TT_END, "end"));

	//Terminal symbols
	auto matchKeywordEnd = 		std::make_shared<edsl::parser::Terminal>(TT_END);
	auto matchKeywordEvents = 	std::make_shared<edsl::parser::Terminal>(TT_EVENT);
	auto matchIdentifier = 		std::make_shared<edsl::parser::Terminal>(TT_IDENTIFIER);

	auto matchEventDec = 		std::make_shared<edsl::parser::SequenceCombinator>(matchIdentifier, matchIdentifier)
									->withSemanticProccessor(std::make_shared<edsl::abstract_syntax_tree::ASTProcessor>("EVENT")->wContent(0)->wContent(1));
	auto matchEventDecList =	std::make_shared<edsl::parser::ListCombinator>(matchEventDec)
									->withSemanticProccessor(std::make_shared<edsl::abstract_syntax_tree::ASTProcessor>("EVENT_LIST"));;
	auto matchEventBlock = 		std::make_shared<edsl::parser::SequenceCombinator>(matchKeywordEvents, matchEventDecList, matchKeywordEnd)
									->withSemanticProccessor(std::make_shared<edsl::abstract_syntax_tree::ASTProcessor>("*")->wStar(1));


	//Running the parser
	edsl::lexer::TokenBuffer tokenbuffer(tokenstream, true);
	edsl::parser::ParseResult result(true, tokenbuffer);
	auto pase_status = matchEventBlock->recognizer(result);
	EXPECT_TRUE(pase_status.isOk());
}
