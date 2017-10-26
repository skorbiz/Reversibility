/*
 * ParseResult.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_PARSERESULT_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_PARSERESULT_H_

#include <reversible_parser/lexer/TokenBuffer.h>
#include <reversible_parser/parser/Error23.h>
#include <memory>

namespace edsl {
namespace abstract_syntax_tree {
class ASTNode;
} /* namespace abstract_syntax_tree */
} /* namespace edsl */

namespace edsl {
namespace parser {

class ParseResult {
public:
	ParseResult(bool grammarOk, lexer::TokenBuffer tokenbuffer, std::string matchType = "");
	ParseResult(bool grammarOk, lexer::TokenBuffer tokenbuffer, lexer::Token matchToken, std::string matchType = "");
	ParseResult(bool grammarOk, lexer::TokenBuffer tokenbuffer, std::shared_ptr<lexer::Token> matchToken, std::string matchType = "");
	virtual ~ParseResult();

	bool isOk();
	std::string getMatchType();
	std::shared_ptr<lexer::Token> getMatchToken();
	lexer::TokenBuffer getTokenBuffer();

	std::shared_ptr<abstract_syntax_tree::ASTNode> getActionResult();
	void setActionResult(std::shared_ptr<abstract_syntax_tree::ASTNode> actResult);

	std::shared_ptr<Error23> getError();
	void setError(std::shared_ptr<Error23> e);

	void to_screen();

private:
	bool grammarOk;
	std::string matchType;
	std::shared_ptr<lexer::Token> matchToken;
	std::shared_ptr<abstract_syntax_tree::ASTNode> actionResult;
	lexer::TokenBuffer tokens;
	std::shared_ptr<Error23> error;



//    private final boolean grammarOk;
//    private final TokenBuffer tokens;
//    private Error error;
//    private Token matchToken;


};

} /* namespace parser */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_PARSERESULT_H_ */
