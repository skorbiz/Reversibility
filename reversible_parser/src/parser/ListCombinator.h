/*
 * ListCombinator.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_LISTCOMBINATOR_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_LISTCOMBINATOR_H_

#include <parser/Combinator.h>
#include <memory>

namespace edsl {
namespace parser {
class ParseResult;
} /* namespace parser */
} /* namespace edsl */

namespace edsl {
namespace parser {

class ListCombinator : public Combinator
{
public:
	ListCombinator(std::shared_ptr<Combinator> production);
	ListCombinator(std::string matchType, std::shared_ptr<Combinator> production);
	virtual ~ListCombinator();

	ParseResult recognizer(ParseResult inbound) const;

private:
	std::string matchType;
	std::shared_ptr<Combinator> production;

};

} /* namespace parser */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_LISTCOMBINATOR_H_ */
