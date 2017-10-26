/*
 * SequenceCombinator.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_SEQUENCECOMBINATOR_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_SEQUENCECOMBINATOR_H_

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

class SequenceCombinator : public Combinator
{

public:
	SequenceCombinator(std::shared_ptr<Combinator> production1);
	SequenceCombinator(std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2);
	SequenceCombinator(std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3);
	SequenceCombinator(std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3, std::shared_ptr<Combinator> production4);
	SequenceCombinator(std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3, std::shared_ptr<Combinator> production4, std::shared_ptr<Combinator> production5);
	SequenceCombinator(std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3, std::shared_ptr<Combinator> production4, std::shared_ptr<Combinator> production5, std::shared_ptr<Combinator> production6);

	SequenceCombinator(std::string match_type, std::shared_ptr<Combinator> production1);
	SequenceCombinator(std::string match_type, std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2);
	SequenceCombinator(std::string match_type, std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3);
	SequenceCombinator(std::string match_type, std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3, std::shared_ptr<Combinator> production4);
	SequenceCombinator(std::string match_type, std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3, std::shared_ptr<Combinator> production4, std::shared_ptr<Combinator> production5);
	SequenceCombinator(std::string match_type, std::shared_ptr<Combinator> production1, std::shared_ptr<Combinator> production2, std::shared_ptr<Combinator> production3, std::shared_ptr<Combinator> production4, std::shared_ptr<Combinator> production5, std::shared_ptr<Combinator> production6);
	virtual ~SequenceCombinator();

	ParseResult recognizer(ParseResult inbound) const;

private:
	std::string matchType;
	std::vector<std::shared_ptr<Combinator> > productions;

	static const std::string matchTypeDefault;

};

} /* namespace parser */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_SEQUENCECOMBINATOR_H_ */
