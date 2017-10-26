/*
 * ASTNode.h
 *
 *  Created on: Jun 6, 2017
 *      Author: josl
 */

#ifndef D7PC_ROS_REVERSIBLE_DEMO_SRC_AST_ASTNODE_H_
#define D7PC_ROS_REVERSIBLE_DEMO_SRC_AST_ASTNODE_H_

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace edsl {
namespace lexer {
class Token;
} /* namespace lexer */
} /* namespace edsl */

namespace edsl {
namespace abstract_syntax_tree {

class ASTNode
{

public:
	ASTNode();
	virtual ~ASTNode();

	size_t size() const;
	std::string getContent() const;
	std::string getContent(unsigned int child_index) const;
	std::string getType() const;
	std::string getType(unsigned int child_index) const;

	std::shared_ptr<ASTNode> child(unsigned int child_index);
	std::shared_ptr<ASTNode> getChild(unsigned int child_inde);
	std::shared_ptr<ASTNode> getSoleChild();


	std::shared_ptr<ASTNode> getSoleChild(std::string type);
	std::shared_ptr<ASTNode> getSoleDecendent(std::string type);
	std::vector<std::shared_ptr<ASTNode> > getChildren(std::string type);
	std::vector<std::shared_ptr<ASTNode> > getDecendents(std::string type);

	std::shared_ptr<ASTNode> getSoleChildWithContent(std::string payload);
	std::shared_ptr<ASTNode> getSoleDecendentWithContent(std::string payload);
	std::vector<std::shared_ptr<ASTNode> > getChildrenWithContent(std::string payload);
	std::vector<std::shared_ptr<ASTNode> > getDecendentsWithContent(std::string payload);

    friend std::ostream& operator<<(std::ostream& os, const ASTNode& t);

    void eraseChild(unsigned int child_index);
    void replaceChild(std::shared_ptr<ASTNode> from, std::shared_ptr<ASTNode> to);


public:

	std::string type;
	std::shared_ptr<lexer::Token> content;
	std::weak_ptr<ASTNode> abstractSyntaxTreeParent;
	std::vector<std::shared_ptr<ASTNode> > abstractSyntaxTreeChilds;
};

} /* namespace abstract_syntax_tree */
} /* namespace edsl */

#endif /* D7PC_ROS_REVERSIBLE_DEMO_SRC_AST_ASTNODE_H_ */
