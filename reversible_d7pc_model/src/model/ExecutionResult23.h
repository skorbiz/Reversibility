/*
 * ExecutionResult23.h
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_EXECUTIONRESULT23_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_EXECUTIONRESULT23_H_

#include <string>

namespace model {

class ExecutionResult23
{
public:
	ExecutionResult23();
	virtual ~ExecutionResult23();

	bool was_execution_succesfull;
	std::string error_msg;
};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_EXECUTIONRESULT23_H_ */
