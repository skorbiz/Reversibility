/*
 * Nothing.h
 *
 *  Created on: Jun 22, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_NOTHING_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_NOTHING_H_

#include <model/InstructionExecutable.h>

namespace model {

class Nothing : public InstructionExecutable
{
public:
	Nothing();
	virtual ~Nothing();

	virtual ExecutionResult23 execute();
	virtual ExecutionResult23 executeBackwards();

};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_NOTHING_H_ */
