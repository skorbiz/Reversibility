/*
 * RestartPoint.h
 *
 *  Created on: Jun 22, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_RESTARTPOINT_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_RESTARTPOINT_H_

#include <model/InstructionExecutable.h>

namespace model {

class RestartPoint : public InstructionExecutable
{

public:
	RestartPoint();
	virtual ~RestartPoint();

	ExecutionResult23 execute();
	ExecutionResult23 executeBackwards();


};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_RESTARTPOINT_H_ */
