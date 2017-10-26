/*
 * PrepareFeeder.h
 *
 *  Created on: Jun 21, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_PREPAREFEEDER_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_PREPAREFEEDER_H_

#include <model/InstructionExecutable.h>
#include <model/Variable.h>
#include <memory>
#include <string>

class AnyfeederPickSkill;

namespace model {

class PrepareFeeder : public InstructionExecutable
{

public:
//	PrepareFeeder(std::shared_ptr<AnyfeederPickSkill> interface, Variable<std::string> object);
	PrepareFeeder(std::shared_ptr<AnyfeederPickSkill> interface, std::string variabel_object);
	virtual ~PrepareFeeder();

	virtual ExecutionResult23 execute();
	virtual ExecutionResult23 executeBackwards();

	std::shared_ptr<AnyfeederPickSkill> anyfeederPickSkill;
	std::string variabel_object;
//	Variable<std::string> object;

};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_PREPAREFEEDER_H_ */
