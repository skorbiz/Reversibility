/*
 * Declaration.h
 *
 *  Created on: Jun 20, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_DECLARATION_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_DECLARATION_H_

#include <model/InstructionExecutable.h>
#include <memory>
#include <string>

namespace model {
class D7pcWorkspace23;
} /* namespace model */

namespace model
{

class Declaration
{

public:
	Declaration(std::shared_ptr<D7pcWorkspace23> wc, std::string varName, std::string varType);
	virtual ~Declaration();
	virtual ExecutionResult23 create(std::shared_ptr<Variables> variables);
	virtual ExecutionResult23 remove(std::shared_ptr<Variables> variables);
private:
	std::shared_ptr<D7pcWorkspace23> wc;
	std::string varName;
	std::string varType;

};



class Local : public Declaration, public InstructionExecutable{
public:
	Local(std::shared_ptr<D7pcWorkspace23> wc, std::string varName, std::string varType);
	virtual ~Local();
	virtual ExecutionResult23 execute();
	virtual ExecutionResult23 executeBackwards();
};

class Delocal : public Declaration, public InstructionExecutable{
public:
	Delocal(std::shared_ptr<D7pcWorkspace23> wc, std::string varName, std::string varType);
	virtual ~Delocal();
	virtual ExecutionResult23 execute();
	virtual ExecutionResult23 executeBackwards();
};

} /* namespace model */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_MODEL_DERIVEDINSTRUCTIONS_DECLARATION_H_ */
