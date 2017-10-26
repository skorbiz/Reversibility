/*
 * ModelFactory.h
 *
 *  Created on: Sep 27, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_D7PC_MODEL_SRC_MODELFACTORY_H_
#define D7PC_ROS_REVERSIBLE_D7PC_MODEL_SRC_MODELFACTORY_H_

#include <memory>
#include <string>

namespace model {
class D7pcWorkspace23;
class Instruction;
class InstructionExecutable;
class Variables;
} /* namespace model */

namespace model {

class ModelFactory
{

public:
	ModelFactory(std::shared_ptr<model::D7pcWorkspace23> workcell, std::shared_ptr<model::Variables> variable_container);
	virtual ~ModelFactory();

	void connectInstructions(std::shared_ptr<model::Instruction> i1, std::shared_ptr<model::Instruction> i2);

	std::shared_ptr<model::Instruction> createInstruction();

	std::shared_ptr<model::InstructionExecutable> createPrint(std::string variable);
	std::shared_ptr<model::InstructionExecutable> createPrepareFeeder(std::string var);
	std::shared_ptr<model::InstructionExecutable> createRestartPoint();
	std::shared_ptr<model::InstructionExecutable> createLocal(std::string varName, std::string type);
	std::shared_ptr<model::InstructionExecutable> createDelocal(std::string varName, std::string type);
	std::shared_ptr<model::InstructionExecutable> createAssignment(std::string name, std::string value);
	std::shared_ptr<model::InstructionExecutable> createUpdate(std::string varName, std::string var);
	std::shared_ptr<model::InstructionExecutable> createNothing();


private:
	std::shared_ptr<model::Variables> variable_container;
	std::shared_ptr<model::D7pcWorkspace23> workcell;


};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_D7PC_MODEL_SRC_MODELFACTORY_H_ */
