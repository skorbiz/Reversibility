/*
 * AST2Graphviz.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_AST_AST2GRAPHVIZ_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_AST_AST2GRAPHVIZ_H_

#include <memory>
#include <string>

namespace edsl {
namespace abstract_syntax_tree {
class ASTNode;
class GraphvizPrinter;
} /* namespace abstract_syntax_tree */
} /* namespace edsl */

namespace edsl {
namespace abstract_syntax_tree {

class AST2Graphviz {
public:
	AST2Graphviz();
	virtual ~AST2Graphviz();

	static void print(std::shared_ptr<GraphvizPrinter> dotAstGraph, std::shared_ptr<ASTNode> root);

private:
	static std::string id(std::shared_ptr<ASTNode> node);

};

} /* namespace abstract_syntax_tree */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_AST_AST2GRAPHVIZ_H_ */
