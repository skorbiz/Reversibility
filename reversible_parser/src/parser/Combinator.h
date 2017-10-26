/*
 * Combinator.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_COMBINATOR_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_COMBINATOR_H_

#include <parser/ParseResult.h>
#include <memory>
#include <vector>

namespace edsl {
namespace parser {
class SemanticProcessor;
} /* namespace parser */
} /* namespace edsl */

namespace edsl {
namespace parser {

class Combinator : public std::enable_shared_from_this<Combinator>
{
public:
	Combinator();
	virtual ~Combinator();
	std::shared_ptr<Combinator> withSemanticProccessor(std::shared_ptr<SemanticProcessor> processor);

	virtual ParseResult recognizer(ParseResult inbound) const = 0;
	void action(ParseResult & result, std::vector<ParseResult> steps = std::vector<ParseResult>()) const;

protected:
	std::shared_ptr<SemanticProcessor> processor;

};

} /* namespace parser */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_COMBINATOR_H_ */






