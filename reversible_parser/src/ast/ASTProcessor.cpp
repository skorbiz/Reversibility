/*
 * ASTProcessor.cpp
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#include <assert.h>
#include <ast/ASTNode.h>
#include <ast/ASTProcessor.h>
#include <lexer/Token.h>
#include <lexer/TokenType.h>
#include <iostream>
#include <utility>

namespace edsl {
namespace abstract_syntax_tree {

const std::string ASTProcessor::rCollapse("");
const std::string ASTProcessor::rStar("*");
const std::string ASTProcessor::rContent("$content");
const std::string ASTProcessor::rType("$type");

ASTProcessor::ASTProcessor()
{
}

ASTProcessor::ASTProcessor(std::string type_override) :
			typeOverride(type_override)
{
}

ASTProcessor::~ASTProcessor()
{
}

std::shared_ptr<ASTProcessor> ASTProcessor::wStar(int i)
{
	Rule rule(i, rStar);
	rules.push_back(rule);
	return shared_from_this();
}

std::shared_ptr<ASTProcessor> ASTProcessor::wContent(int i)
{
	Rule rule(i, rContent);
	rules.push_back(rule);
	return shared_from_this();
}

void ASTProcessor::handleParseResult(std::vector<parser::ParseResult> & subresults, parser::ParseResult & result) const
{

	std::shared_ptr<ASTNode> ast = std::make_shared<ASTNode>();

	ast->type = getNodeType(result);
	ast->content = result.getMatchToken();
	ast->abstractSyntaxTreeChilds = createChildren(subresults);

	for(auto & child : ast->abstractSyntaxTreeChilds)
		child->abstractSyntaxTreeParent = ast;

	result.setActionResult(ast);
}

std::string ASTProcessor::getNodeType(parser::ParseResult result) const
{
	if(typeOverride == "")
		return result.getMatchType();
	else if(typeOverride == "$")
	{
		assert(result.getMatchToken() != nullptr);
		return result.getMatchToken()->type.type;
	}
	return typeOverride;
}

std::vector<std::shared_ptr<ASTNode> > ASTProcessor::createChildren(std::vector<parser::ParseResult> results) const
{
	std::vector<std::shared_ptr<ASTNode> > children;

	if(rules.size() != 0)
		for(unsigned int i = 0; i < rules.size(); i++)
			children.push_back(createChildFromRule(results, rules[i]));
	else
		for(unsigned int i = 0; i < results.size(); i++)
			children.push_back(createChildFromRule(results, Rule(i, rStar)));

	//Checks for Nodes to collaps
	std::vector<std::shared_ptr<ASTNode> > children_collapsed;
	for(unsigned int i = 0; i < children.size(); i++)
		if(children[i]->type == "" || children[i]->type == "OR_COMBINATOR")
			children_collapsed.insert( children_collapsed.end(), children[i]->abstractSyntaxTreeChilds.begin(), children[i]->abstractSyntaxTreeChilds.end() );
		else
			children_collapsed.push_back(children[i]);
	return children_collapsed;
}

std::shared_ptr<ASTNode> ASTProcessor::createChildFromRule(std::vector<parser::ParseResult> results, Rule rule_input) const
{

	assert(rule_input.first < (int) results.size());

	int n = rule_input.first;
	std::string rule = rule_input.second;
	std::shared_ptr<lexer::Token> token = results[n].getMatchToken();
	std::shared_ptr<ASTNode> ast = std::make_shared<ASTNode>();

	if(rule ==  rStar)
	{
		ast = results[n].getActionResult();
	}
	else if(rule == rContent)
	{
		assert(token != nullptr);
		ast->type = token->payload;
		ast->content = token;
	}
	else if(rule ==  rType)
	{
		assert(token != nullptr);
		ast->type = token->type.type;
		ast->content = token;
	}

	return ast;
}



} /* namespace abstract_syntax_tree */
} /* namespace edsl */
