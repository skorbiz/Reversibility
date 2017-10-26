/*
 * Update.h
 *
 *  Created on: Jun 22, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_UPDATE_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_UPDATE_H_

#include <model/InstructionExecutable.h>
#include <model/Variable.h>
#include <rw/kinematics/State.hpp>
#include <memory>
#include <string>

namespace model {
class D7pcWorkspace23;
} /* namespace model */

namespace model
{

class Update : public InstructionExecutable
{
public:
	Update(std::string variable);
	virtual ~Update();

	virtual ExecutionResult23 execute();
	virtual ExecutionResult23 executeBackwards();

private:
	std::shared_ptr<D7pcWorkspace23> wc;
	std::string var;

};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_UPDATE_H_ */
