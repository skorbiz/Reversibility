/*
 * D7pcParser.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <ast/ASTProcessor.h>
#include <D7pcAstType.h>
#include <D7pcParser.h>
#include <D7pcTokenType.h>
#include <lexer/TokenType.h>
#include <parser/Error23.h>
#include <parser/ListCombinator.h>
#include <parser/OrCombinator.h>
#include <parser/ParseResult.h>
#include <parser/SequenceCombinator.h>
#include <parser/Terminal.h>
#include <iostream>
#include <memory>

namespace edsl {

D7pcParser::D7pcParser()
{
}

D7pcParser::~D7pcParser()
{
}



parser::ParseResult D7pcParser::parse(lexer::TokenBuffer tokenbuffer)
{
	using namespace edsl::lexer;
	using namespace edsl::parser;
	using namespace edsl::abstract_syntax_tree;


	//Terminal symbols
	std::shared_ptr<Combinator> kSequence = std::make_shared<Terminal>(D7pcTokenType::TT_SEQUENCE);
	std::shared_ptr<Combinator> kFrames = 	std::make_shared<Terminal>(D7pcTokenType::TT_FRAMES);
	std::shared_ptr<Combinator> kJointq = 	std::make_shared<Terminal>(D7pcTokenType::TT_JOINTQ);
	std::shared_ptr<Combinator> end = 		std::make_shared<Terminal>(D7pcTokenType::TT_END);

	std::shared_ptr<Combinator> kcall = 	std::make_shared<Terminal>(D7pcTokenType::TT_CALL);
	std::shared_ptr<Combinator> kuncall = 	std::make_shared<Terminal>(D7pcTokenType::TT_UNCALL);
	std::shared_ptr<Combinator> klocal = 	std::make_shared<Terminal>(D7pcTokenType::TT_LOCAL);
	std::shared_ptr<Combinator> kdelocal = 	std::make_shared<Terminal>(D7pcTokenType::TT_DELOCAL);

	std::shared_ptr<Combinator> left = 		std::make_shared<Terminal>(D7pcTokenType::TT_LEFT);
	std::shared_ptr<Combinator> right = 	std::make_shared<Terminal>(D7pcTokenType::TT_RIGHT);
	std::shared_ptr<Combinator> dot = 		std::make_shared<Terminal>(D7pcTokenType::TT_DOT);
	std::shared_ptr<Combinator> comma = 	std::make_shared<Terminal>(D7pcTokenType::TT_COMMA);
	std::shared_ptr<Combinator> equal =		std::make_shared<Terminal>(D7pcTokenType::TT_EQUAL);
	std::shared_ptr<Combinator> colon = 	std::make_shared<Terminal>(D7pcTokenType::TT_COLON);

	std::shared_ptr<Combinator> text = 				std::make_shared<Terminal>(D7pcTokenType::TT_STRING);
	std::shared_ptr<Combinator> identifier = 		std::make_shared<Terminal>(D7pcTokenType::TT_IDENTIFIER);

	//Nonterminal production rules
	std::shared_ptr<Combinator> compoundIdentifier = std::make_shared<SequenceCombinator>(identifier, dot, nullptr);

	std::shared_ptr<Combinator> frames = std::make_shared<ListCombinator>(compoundIdentifier)
			->withSemanticProccessor(std::make_shared<ASTProcessor>(D7pcAstType::AT_FRAMES));
	std::shared_ptr<Combinator> listFrames = std::make_shared<SequenceCombinator>(kFrames, frames, end)
			->withSemanticProccessor(std::make_shared<ASTProcessor>("lFRAMES")->wStar(1));

	std::shared_ptr<Combinator> joints =		std::make_shared<ListCombinator>(compoundIdentifier);
	std::shared_ptr<Combinator> listJoints =	std::make_shared<SequenceCombinator>(kJointq, joints, end);



	std::shared_ptr<Combinator> arg1 = std::make_shared<SequenceCombinator>(identifier, dot, nullptr);
	std::shared_ptr<Combinator> arg = std::make_shared<OrCombinator>(D7pcAstType::AT_ARGUMENT,arg1, text);
	std::shared_ptr<Combinator> arg_list( new SequenceCombinator(arg, comma, nullptr));
	std::shared_ptr<Combinator> cmd2 = std::make_shared<SequenceCombinator>(identifier, left, right);
	std::shared_ptr<Combinator> cmd1 = std::make_shared<SequenceCombinator>(identifier, left, arg_list, right);
	std::shared_ptr<Combinator> expression( new OrCombinator({cmd1, cmd2, arg, text}));

	std::shared_ptr<Combinator> uncall = std::make_shared<SequenceCombinator>(kuncall, left, identifier, right);
	std::shared_ptr<Combinator> local1 = std::make_shared<SequenceCombinator>(klocal, identifier, identifier, equal, expression);
	std::shared_ptr<Combinator> local2 = std::make_shared<SequenceCombinator>(klocal, identifier, identifier);
	std::shared_ptr<Combinator> delocal1 = std::make_shared<SequenceCombinator>(kdelocal, identifier, identifier, equal, expression);
	std::shared_ptr<Combinator> delocal2 = std::make_shared<SequenceCombinator>(kdelocal, identifier, identifier);
	std::shared_ptr<Combinator> assignment = std::make_shared<SequenceCombinator>(identifier, equal, expression);
	std::shared_ptr<Combinator> update = std::make_shared<SequenceCombinator>(identifier, colon, equal, expression);
	std::shared_ptr<Combinator> print = std::make_shared<SequenceCombinator>(identifier, text);
	std::shared_ptr<Combinator> statement(new OrCombinator({uncall, local1, local2, delocal1, delocal2, assignment, update, print, expression}));
	std::shared_ptr<Combinator> statement_list = std::make_shared<ListCombinator>(D7pcAstType::AT_INSTRUCTION_LIST, statement);


	std::shared_ptr<Combinator> sequence = std::make_shared<SequenceCombinator>(D7pcTokenType::TT_SEQUENCE.type, kSequence, identifier, statement_list, end)
		->withSemanticProccessor(std::make_shared<ASTProcessor>()->wStar(1)->wStar(2));

	std::shared_ptr<Combinator> lists (new OrCombinator({listFrames, listJoints, sequence}));
	std::shared_ptr<Combinator> main (new ListCombinator(lists));

	//Running the Combinators
	ParseResult input(true, tokenbuffer);
	auto result = main->recognizer(input);


	if(!result.isOk())
	{
		std::cout << "Warning: D7pcParser failed to pass input" << std::endl;
		result.getError()->to_screen();
	}

	if(result.getTokenBuffer().size() > 0)
	{
		std::cout << "Warning: D7pcParser failed to pass all tokens" << std::endl;
		std::cout << "         Token buffer still has size: " << result.getTokenBuffer().size() << std::endl;
		std::cout << "         Next unpassed token was: " << result.getTokenBuffer().peakNextToken() << std::endl;
	}
	return result;
}


} /* namespace edsl */
