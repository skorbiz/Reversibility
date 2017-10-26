/*
 * Print.h
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_PRINT_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_PRINT_H_

#include <model/InstructionExecutable.h>
#include <model/Variable.h>
#include <memory>
#include <string>
#include <model/ExecutionResult23.h>

namespace model {

class Print : public InstructionExecutable
{
public:
	Print(std::string variabel_name);
	virtual ~Print();

	ExecutionResult23 execute();
	ExecutionResult23 executeBackwards();

private:

	std::string variabel_name;

};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_PRINT_H_ */
