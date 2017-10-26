/*
 * Error23.h
 *
 *  Created on: Jun 7, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_ERROR23_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_ERROR23_H_

#include <memory>
#include <string>

namespace edsl {
namespace parser {

class Error23
{

public:
	Error23();
	Error23(std::string msg, int line);
	Error23(std::string msg, int line, std::shared_ptr<Error23> suberror);
	virtual ~Error23();

	void to_screen();

	std::string message;
	int line;
	std::shared_ptr<Error23> suberror;

};

} /* namespace parser */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_PARSER_ERROR23_H_ */
