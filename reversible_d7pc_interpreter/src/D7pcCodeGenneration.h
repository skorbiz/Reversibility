/*
 * D7pcCodeGenneration.h
 *
 *  Created on: Jun 12, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PCCODEGENNERATION_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PCCODEGENNERATION_H_

#include <memory>
#include <reversible_parser/ast/ASTNode.h>

namespace edsl {

class D7pcCodeGenneration
{

public:
	D7pcCodeGenneration(std::shared_ptr<abstract_syntax_tree::ASTNode> root);
	virtual ~D7pcCodeGenneration();

private:
	std::string toString(std::shared_ptr<abstract_syntax_tree::ASTNode> identifier);

private:
	std::shared_ptr<abstract_syntax_tree::ASTNode> root;

};

} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_D7PCCODEGENNERATION_H_ */
