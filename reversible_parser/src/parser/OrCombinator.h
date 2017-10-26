/*
 * OrCombinator.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_ORCOMBINATOR_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_ORCOMBINATOR_H_

#include <parser/Combinator.h>
#include <memory>
#include <vector>

namespace edsl {
namespace parser {
class ParseResult;
} /* namespace parser */
} /* namespace edsl */

namespace edsl {
namespace parser {

class OrCombinator : public Combinator
{

public:
	OrCombinator(std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2);
	OrCombinator(std::initializer_list<std::shared_ptr<Combinator> >input);
	OrCombinator(std::string matchType, std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2);
	OrCombinator(std::string matchType, std::initializer_list<std::shared_ptr<Combinator> >input);

	virtual ~OrCombinator();

	ParseResult recognizer(ParseResult inbound) const;

private:
	std::string matchType;
	std::vector<std::shared_ptr<Combinator> > productions;

};

} /* namespace parser */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_ORCOMBINATOR_H_ */
