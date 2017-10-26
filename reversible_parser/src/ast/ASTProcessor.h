/*
 * ASTProcessor.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_AST_ASTPROCESSOR_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_AST_ASTPROCESSOR_H_

#include <parser/ParseResult.h>
#include <parser/SemanticProcessor.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace edsl {
namespace abstract_syntax_tree {

class ASTProcessor : public parser::SemanticProcessor, public std::enable_shared_from_this<ASTProcessor>
{
	typedef std::pair<int, std::string> Rule;

public:
	ASTProcessor();
	ASTProcessor(std::string type_override);
	virtual ~ASTProcessor();

	std::shared_ptr<ASTProcessor> wStar(int i);
	std::shared_ptr<ASTProcessor> wContent(int i);

	void handleParseResult(std::vector<parser::ParseResult> & subresults, parser::ParseResult & result) const;


private:
	std::vector<std::shared_ptr<ASTNode> > createChildren(std::vector<parser::ParseResult> results) const;
	std::shared_ptr<ASTNode> createChildFromDefault(std::vector<parser::ParseResult> results) const;
	std::shared_ptr<ASTNode> createChildFromRule(std::vector<parser::ParseResult> results, Rule rule) const;

	std::string getNodeType(parser::ParseResult result) const;


private:
	std::string typeOverride;
	std::vector<Rule> rules;


	static const std::string rCollapse;
	static const std::string rStar;
	static const std::string rContent;
	static const std::string rType;

};

} /* namespace abstract_syntax_tree */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_AST_ASTPROCESSOR_H_ */
