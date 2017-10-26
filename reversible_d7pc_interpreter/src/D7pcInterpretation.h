/*
 * D7pcInterpretation.h
 *
 *  Created on: Jun 15, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PCINTERPRETATION_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PCINTERPRETATION_H_

#include <memory>
#include <string>

namespace model {
class ModelFactory;
} /* namespace model */

namespace model {
class D7pcWorkspace23;
} /* namespace model */

class D7cpWorkcell;

namespace model {
class InstructionExecutable;
} /* namespace model */

namespace model {
class Variables;
} /* namespace model */

namespace edsl {
namespace abstract_syntax_tree {
class ASTNode;
} /* namespace abstract_syntax_tree */
} /* namespace edsl */
namespace model {
class Instruction;
} /* namespace model */

namespace edsl {

class D7pcInterpretation {
public:
	D7pcInterpretation(std::shared_ptr<model::ModelFactory> factory);
	virtual ~D7pcInterpretation();

	struct Build_return_type
	{
		std::shared_ptr<model::Instruction> start;
		std::shared_ptr<model::Instruction> end;
	};

	Build_return_type interprete(std::shared_ptr<abstract_syntax_tree::ASTNode> root);


private:
	std::shared_ptr<model::ModelFactory> factory;
	std::shared_ptr<abstract_syntax_tree::ASTNode> root;

	Build_return_type build(std::shared_ptr<abstract_syntax_tree::ASTNode> node);
	Build_return_type build_sequence(std::string name, std::shared_ptr<abstract_syntax_tree::ASTNode> node);
	Build_return_type build_terminal(std::shared_ptr<abstract_syntax_tree::ASTNode> node);



};



// ******************************************************************
// **** TEMPLATE IMPLEMENTATIONS ************************************
// ******************************************************************



} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PCINTERPRETATION_H_ */
