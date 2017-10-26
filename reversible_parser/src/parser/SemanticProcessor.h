/*
 * SemanticProcessor.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_SEMANTICPROCESSOR_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_SEMANTICPROCESSOR_H_

#include <parser/ParseResult.h>
#include <memory>
#include <vector>


namespace edsl {
namespace parser {

class SemanticProcessor {
public:
	SemanticProcessor();
	virtual ~SemanticProcessor();

	virtual void handleParseResult(std::vector<ParseResult> & steps, ParseResult & result) const;

};

} /* namespace parser */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_SEMANTICPROCESSOR_H_ */
